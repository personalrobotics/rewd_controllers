#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/Spline.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <boost/make_shared.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pluginlib/class_list_macros.h>
#include <rewd_controllers/JointTrajectoryController.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace rewd_controllers {

//=============================================================================
JointTrajectoryController::JointTrajectoryController()
{
  using hardware_interface::EffortJointInterface;
  using hardware_interface::PositionJointInterface;
  using hardware_interface::VelocityJointInterface;

  mAdapterFactory.registerFactory<
    PositionJointInterface, JointPositionAdapter>("position");
  mAdapterFactory.registerFactory<
    VelocityJointInterface, JointVelocityAdapter>("velocity");
  mAdapterFactory.registerFactory<
    EffortJointInterface, JointEffortAdapter>("effort");
}

//=============================================================================
JointTrajectoryController::~JointTrajectoryController()
{
}

//=============================================================================
bool JointTrajectoryController::init(
  hardware_interface::RobotHW *robot, ros::NodeHandle &n)
{
  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using hardware_interface::JointStateInterface;

  // Build up the list of controlled DOFs.
  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto& param : jointParameters)
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton)
      return false;

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton = getControlledMetaSkeleton(
    mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton)
    return false;

  mControlledSpace = std::make_shared<MetaSkeletonStateSpace>(
    mControlledSkeleton);

  // Retreive JointStateHandles required to update the position and velocity of
  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface)
  {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
    new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  // Create adaptors to provide a uniform interface to different types.
  const ros::NodeHandle gainsNodeHandle{n, "gains"};
  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mAdapters.resize(mControlledSkeleton->getNumDofs());

  for (size_t idof = 0; idof < numControlledDofs; ++idof)
  {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter)
      return false;

    // Initialize the adapter using parameters stored on the parameter server.
    ros::NodeHandle adapterNodeHandle{gainsNodeHandle, dof->getName()};
    if (!adapter->initialize(adapterNodeHandle))
      return false;

    mAdapters[idof] = std::move(adapter);
  }

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mDesiredAcceleration.resize(numControlledDofs);
  mNominalForce.resize(numControlledDofs);

  // Start the action server. This must be last.
  mActionServer.reset(
    new actionlib::ActionServer<Action>{n, "follow_joint_trajectory",
      boost::bind(&JointTrajectoryController::goalCallback, this, _1),
      boost::bind(&JointTrajectoryController::cancelCallback, this, _1),
      false});
  mActionServer->start();

  return true;
}

//=============================================================================
void JointTrajectoryController::starting(const ros::Time& time)
{
  mSkeletonUpdater->update();

  // Hold the current position.
  mDesiredPosition = mControlledSkeleton->getPositions();
  mDesiredVelocity.setZero();
  mDesiredAcceleration.setZero();

  ROS_INFO_STREAM("Initialized desired position: "
    << mDesiredPosition.transpose());
  ROS_INFO_STREAM("Initialized desired velocity: "
    << mDesiredVelocity.transpose());
  ROS_INFO_STREAM("Initialized desired acceleration: "
    << mDesiredAcceleration.transpose());

  // Reset any internal state in the adapters (e.g. integral windup).
  for (const auto& adapter : mAdapters)
    adapter->reset();

  ROS_INFO("Reset joint adapters.");
}

//=============================================================================
void JointTrajectoryController::stopping(const ros::Time& time)
{
  mActionServer.reset();
}

//=============================================================================
void JointTrajectoryController::update(
  const ros::Time& time, const ros::Duration& period)
{
  // TODO: Make this a member variable to avoid dynamic allocation.
  auto mDesiredState = mControlledSpace->createState();

  std::shared_ptr<TrajectoryContext> context;
  mCurrentTrajectory.get(context);

  if (context)
  {
    const auto& trajectory = context->mTrajectory;
    const auto timeFromStart = std::min(
      (time - context->mStartTime).toSec(), trajectory->getEndTime());

    // Evaluate the trajectory at the current time.
    trajectory->evaluate(timeFromStart, mDesiredState);
    mControlledSpace->convertStateToPositions(
      mDesiredState, mDesiredPosition);
    trajectory->evaluateDerivative(timeFromStart, 1, mDesiredVelocity);
    trajectory->evaluateDerivative(timeFromStart, 2, mDesiredAcceleration);

    // TODO: Check path constraints.
    // TODO: Check goal constraints.

    // Terminate the current trajectory.
    if (timeFromStart >= trajectory->getDuration())
    {
      context->mGoalHandle.setSucceeded();
      mCurrentTrajectory.set(nullptr);

      ROS_INFO_STREAM(
        "Finished executing trajectory '"
        << context->mGoalHandle.getGoalID().id
        << "' at time " << time << ".");
    }
  }

  // Compute inverse dynamics torques from the set point and store them in the
  // skeleton. These values may be queried by the adapters below.
  mSkeletonUpdater->update();
  mControlledSkeleton->setPositions(mDesiredPosition);
  mControlledSkeleton->setVelocities(mDesiredVelocity);
  mControlledSkeleton->setAccelerations(mDesiredAcceleration);

  mSkeleton->computeInverseDynamics();
  mNominalForce = mControlledSkeleton->getForces();

  // Restore the state of the Skeleton from JointState interfaces. These values
  // may be used by the adapters below.
  mSkeletonUpdater->update();
  mControlledSkeleton->setForces(mNominalForce);

  for (size_t idof = 0; idof < mAdapters.size(); ++idof)
  {
    mAdapters[idof]->update(
      time, period, mDesiredPosition[idof], mDesiredVelocity[idof]);
  }
}

//=============================================================================
void checkVector(
  const std::string& name, const std::vector<double>& values,
  size_t expectedLength, bool isRequired, Eigen::VectorXd* output)
{
  if (values.empty())
  {
    if (isRequired)
    {
      std::stringstream message;
      message << name << " are required.";
      throw std::runtime_error(message.str());
    }
  }
  else if (values.size() != expectedLength)
  {
    std::stringstream message;
    message << "Expected " << name << " to be of length " << expectedLength
      << ", got " << values.size() << ".";
    throw std::runtime_error(message.str());
  }

  if (output)
    *output = Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

//=============================================================================
void extractJointTrajectoryPoint(
  const trajectory_msgs::JointTrajectory& trajectory,
  size_t index, size_t numDofs,
  Eigen::VectorXd* positions, bool positionsRequired,
  Eigen::VectorXd* velocities, bool velocitiesRequired,
  Eigen::VectorXd* accelerations, bool accelerationsRequired)
{
  const auto& waypoint = trajectory.points[index];

  try
  {
    checkVector("positions", waypoint.positions, numDofs,
      positionsRequired, positions);
    checkVector("velocities", waypoint.velocities, numDofs,
      velocitiesRequired, velocities);
    checkVector("accelerations", waypoint.accelerations, numDofs,
      accelerationsRequired, accelerations);
  }
  catch (const std::runtime_error& e)
  {
    std::stringstream message;
    message << "Waypoint " << index << " is invalid: " << e.what();
    throw std::runtime_error(message.str());
  }
}

//=============================================================================
Eigen::MatrixXd fitPolynomial(
  double currTime,
  const Eigen::VectorXd& currPosition,
  const Eigen::VectorXd& currVelocity,
  const Eigen::VectorXd& currAcceleration,
  double nextTime,
  const Eigen::VectorXd& nextPosition,
  const Eigen::VectorXd& nextVelocity,
  const Eigen::VectorXd& nextAcceleration,
  size_t numCoefficients)
{
  using aikido::util::SplineProblem;

  assert(numCoefficients == 2 || numCoefficients == 4 || numCoefficients == 6);

  const auto numDofs = currPosition.size();
  SplineProblem<> splineProblem(
    Eigen::Vector2d(currTime, nextTime), numCoefficients, numDofs);

  assert(currPosition.size() == numDofs);
  assert(nextPosition.size() == numDofs);
  splineProblem.addConstantConstraint(0, 0, currPosition);
  splineProblem.addConstantConstraint(1, 0, nextPosition);

  if (numCoefficients == 3)
  {
    assert(currVelocity.size() == numDofs);
    assert(nextVelocity.size() == numDofs);
    splineProblem.addConstantConstraint(0, 1, currVelocity);
    splineProblem.addConstantConstraint(1, 1, nextVelocity);
  }

  if (numCoefficients == 6)
  {
    assert(currAcceleration.size() == numDofs);
    assert(nextAcceleration.size() == numDofs);
    splineProblem.addConstantConstraint(0, 2, currAcceleration);
    splineProblem.addConstantConstraint(1, 2, nextAcceleration);
  }

  const auto splineSegment = splineProblem.fit();
  return splineSegment.getCoefficients()[0];
}

std::unique_ptr<aikido::trajectory::Spline> convertJointTrajectory(
  const std::shared_ptr<MetaSkeletonStateSpace>& space,
  const trajectory_msgs::JointTrajectory& jointTrajectory)
{
  using SplineTrajectory = aikido::trajectory::Spline;

  const auto numControlledDofs = space->getNumSubspaces();

  if (jointTrajectory.joint_names.size() != numControlledDofs)
  {
    std::stringstream message;
    message << "Incorrect number of joints: expected "
        << numControlledDofs << ", got "
        << jointTrajectory.joint_names.size() << ".";
    throw std::runtime_error{message.str()};
  }

  if (jointTrajectory.points.empty())
    throw std::runtime_error{"Trajectory contains no waypoints."};

  // Extract the first waypoint to infer the dimensionality of the trajectory.
  Eigen::VectorXd currPosition, currVelocity, currAcceleration;
  extractJointTrajectoryPoint(jointTrajectory, 0, numControlledDofs,
    &currPosition, true, &currVelocity, false, &currAcceleration, false);

  const auto& firstWaypoint = jointTrajectory.points.front();
  auto currTimeFromStart = firstWaypoint.time_from_start.toSec();

  const auto isVelocityRequired = (currVelocity.size() != 0);
  const auto isAccelerationRequired = (currAcceleration.size() != 0);
  if (isAccelerationRequired && !isVelocityRequired)
  {
    throw std::runtime_error{
      "Velocity is required since acceleration is specified."};
  }

  int numCoefficients;
  if (isAccelerationRequired)
    numCoefficients = 6; // quintic
  else if (isVelocityRequired)
    numCoefficients = 4; // cubic
  else
    numCoefficients = 2; // linear;

  // Convert the ROS trajectory message to an Aikido spline.
  std::unique_ptr<SplineTrajectory> trajectory{new SplineTrajectory{space}};
  auto currState = space->createState();

  const auto& waypoints = jointTrajectory.points;
  for (size_t iwaypoint = 1; iwaypoint < waypoints.size(); ++iwaypoint)
  {
    Eigen::VectorXd nextPosition, nextVelocity, nextAcceleration;
    extractJointTrajectoryPoint(jointTrajectory, iwaypoint, numControlledDofs,
      &nextPosition, true, &nextVelocity, isVelocityRequired,
      &nextAcceleration, isAccelerationRequired);

    // Compute spline coefficients for this polynomial segment.
    const auto& nextWaypoint = waypoints[iwaypoint];
    const auto nextTimeFromStart = nextWaypoint.time_from_start.toSec();
    const auto segmentDuration = nextTimeFromStart - currTimeFromStart;
    const auto segmentCoefficients = fitPolynomial(
      0., currPosition, currVelocity, currAcceleration,
      segmentDuration, nextPosition, nextVelocity, nextAcceleration,
      numCoefficients);

    // Add a segment to the trajectory.
    space->convertPositionsToState(currPosition, currState);
    trajectory->addSegment(segmentCoefficients, segmentDuration, currState);

    // Advance to the next segment.
    currPosition = nextPosition;
    currVelocity = nextVelocity;
    currAcceleration = nextAcceleration;
    currTimeFromStart = nextTimeFromStart;
  }

  return std::move(trajectory);
}

//=============================================================================
void JointTrajectoryController::goalCallback(GoalHandle goalHandle)
{
  const auto goal = goalHandle.getGoal();
  ROS_INFO_STREAM("Received trajectory '" << goalHandle.getGoalID().id
    << "' with " << goal->trajectory.points.size() << ".");
  
  // Convert the JointTrajectory message to a format that we can execute.
  std::shared_ptr<aikido::trajectory::Spline> trajectory;
  try
  {
    trajectory = convertJointTrajectory(mControlledSpace, goal->trajectory);
  }
  catch (const std::runtime_error& e)
  {
    Result result;
    result.error_code = Result::INVALID_GOAL;
    result.error_string = e.what();
    goalHandle.setRejected(result);

    ROS_ERROR_STREAM("Rejected trajectory: " << e.what());
    return;
  }

  ROS_INFO_STREAM("Converted to Aikido trajectory with "
    << trajectory->getNumSegments() << " segments and "
    << trajectory->getNumDerivatives() << " derivatives.");

  // Infer the start time of the trajectory.
  const auto now = ros::Time::now();
  const auto specifiedStartTime = goal->trajectory.header.stamp;

  ros::Time startTime;
  if (!specifiedStartTime.isZero())
  {
    startTime = specifiedStartTime;
  }
  else
  {
    startTime = now;
    ROS_WARN("Trajectory does not have an explicit start time, so"
             " we assume that it will start immediately.");
  }

  if (startTime < now)
  {
    std::stringstream message;
    message << "Trajectory starts in the past: " << specifiedStartTime
      << " < " << now << ".";

    Result result;
    result.error_code = Result::OLD_HEADER_TIMESTAMP;
    result.error_string = message.str();
    goalHandle.setRejected(result);

    ROS_ERROR_STREAM("Rejected trajectory: " << message.str());
    return;
  }

  ROS_INFO_STREAM("Trajectory will start at time " << startTime);

  // Preempt the existing trajectory, if one is running.
  std::shared_ptr<TrajectoryContext> existingContext;
  mCurrentTrajectory.get(existingContext);

  if (existingContext)
  {
    auto& existingGoalHandle = existingContext->mGoalHandle;
    ROS_WARN_STREAM("Preempted trajectory '"
      << existingGoalHandle.getGoalID().id << "' with trajectory '"
      << goalHandle.getGoalID().id << "'.");

    // TODO: Make this access thread safe.
    existingGoalHandle.setCanceled();
  }

  // Start the new trajectory.
  const auto newContext = std::make_shared<TrajectoryContext>();
  newContext->mStartTime = startTime;
  newContext->mTrajectory = trajectory;
  newContext->mGoalHandle = goalHandle;

  ROS_INFO_STREAM(
    "Started executing trajectory '" << goalHandle.getGoalID().id
    << "' with duration " << trajectory->getDuration()
    << " at time " << startTime << ".");

  goalHandle.setAccepted();
  mCurrentTrajectory.set(newContext);
}

//=============================================================================
void JointTrajectoryController::cancelCallback(GoalHandle goalHandle)
{
  ROS_WARN("Canceling is not implemented.");
}

} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::JointTrajectoryController,
  controller_interface::ControllerBase)
