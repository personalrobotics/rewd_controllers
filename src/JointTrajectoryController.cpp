#include <rewd_controllers/JointTrajectoryController.hpp>

#include <functional>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <aikido/util/Spline.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pluginlib/class_list_macros.h>

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace rewd_controllers {
namespace {

//=============================================================================
std::vector<double> toVector(const Eigen::VectorXd& input)
{
  return std::vector<double>{input.data(), input.data() + input.size()};
}

} // namespace

//=============================================================================
JointTrajectoryController::JointTrajectoryController()
  : MultiInterfaceController(true)  // allow_optional_interfaces
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
  mAdapters.resize(numControlledDofs);

  for (size_t idof = 0; idof < numControlledDofs; ++idof)
  {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter)
      return false;

    // Initialize the adapter using parameters stored on the parameter server.
    auto dofName = dof->getName();
    if (dofName.at(0) == '/')
      dofName.erase(0, 1);  // a leading slash creates a root level namespace

    // Initialize the adapter using parameters stored on the parameter server.
    ros::NodeHandle adapterNodeHandle{gainsNodeHandle, dofName};
    if (!adapter->initialize(adapterNodeHandle))
      return false;

    mAdapters[idof] = std::move(adapter);
  }

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mDesiredAcceleration.resize(numControlledDofs);
  mDesiredEffort.resize(numControlledDofs);

  // Start the action server. This must be last.
  using std::placeholders::_1;
  mActionServer.reset(
    new actionlib::ActionServer<Action>{n, "follow_joint_trajectory",
        std::bind(&JointTrajectoryController::goalCallback, this, _1),
      std::bind(&JointTrajectoryController::cancelCallback, this, _1),
      false});
  mActionServer->start();

  mNonRealtimeTimer = n.createTimer(
    ros::Duration(0.02), &JointTrajectoryController::nonRealtimeCallback,
    this, false, true);

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
  mNonRealtimeTimer.stop();
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
      // TODO: This should not happen in the realtime thread.
      context->mGoalHandle.setSucceeded();
      mCurrentTrajectory.set(nullptr);

      ROS_INFO_STREAM(
        "Finished executing trajectory '"
        << context->mGoalHandle.getGoalID().id
        << "' at time " << time << ".");
    }
  }

  // Update the state of the Skeleton.
  mSkeletonUpdater->update();
  mActualPosition = mControlledSkeleton->getPositions();
  mActualVelocity = mControlledSkeleton->getVelocities();
  mActualEffort = mControlledSkeleton->getForces();

  // Compute inverse dynamics torques from the set point and store them in the
  // skeleton. These values may be queried by the adapters below.
  mControlledSkeleton->setPositions(mDesiredPosition);
  mControlledSkeleton->setVelocities(mDesiredVelocity);
  mControlledSkeleton->setAccelerations(mDesiredAcceleration);

  mSkeleton->computeInverseDynamics();
  mDesiredEffort = mControlledSkeleton->getForces();

  // Restore the state of the Skeleton from JointState interfaces. These values
  // may be used by the adapters below.
  mControlledSkeleton->setPositions(mActualPosition);
  mControlledSkeleton->setVelocities(mActualVelocity);

  for (size_t idof = 0; idof < mAdapters.size(); ++idof)
  {
    mAdapters[idof]->update(time, period,
      mActualPosition[idof], mDesiredPosition[idof],
      mActualVelocity[idof], mDesiredVelocity[idof],
      mDesiredEffort[idof]);
  }
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
    trajectory = aikido::control::ros::convertJointTrajectory(
      mControlledSpace, goal->trajectory);
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

//=============================================================================
void JointTrajectoryController::nonRealtimeCallback(
  const ros::TimerEvent &event)
{
  std::shared_ptr<TrajectoryContext> context;
  mCurrentTrajectory.get(context);

  if (context)
  {
    const ros::Duration timeFromStart{
      event.current_real - context->mStartTime};

    Feedback feedback;
    feedback.header.stamp = event.current_real; // TODO: Use control loop time.

    for (const auto dof : mControlledSkeleton->getDofs())
      feedback.joint_names.emplace_back(dof->getName());

    feedback.desired.positions = toVector(mDesiredPosition);
    feedback.desired.velocities = toVector(mDesiredVelocity);
    feedback.desired.accelerations = toVector(mDesiredAcceleration);
    feedback.desired.effort = toVector(mDesiredEffort);
    feedback.desired.time_from_start = timeFromStart;
    feedback.actual.positions = toVector(mActualPosition);
    feedback.actual.velocities = toVector(mActualVelocity);
    feedback.actual.effort = toVector(mActualEffort);
    feedback.actual.time_from_start = timeFromStart;
    feedback.error.positions = toVector(mDesiredPosition - mActualPosition);
    feedback.error.velocities = toVector(mDesiredVelocity - mActualVelocity);
    feedback.error.effort = toVector(mDesiredEffort - mActualEffort);
    feedback.error.time_from_start = timeFromStart;

    context->mGoalHandle.publishFeedback(feedback);
  }
}

} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::JointTrajectoryController,
  controller_interface::ControllerBase)
