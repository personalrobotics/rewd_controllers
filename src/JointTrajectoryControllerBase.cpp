#include <rewd_controllers/JointTrajectoryControllerBase.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <aikido/control/ros/Conversions.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/common/Spline.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::R1Joint;

namespace rewd_controllers
{
namespace
{
//=============================================================================
std::vector<double> toVector(const Eigen::VectorXd& input)
{
  return std::vector<double>{input.data(), input.data() + input.size()};
}

}  // namespace

//=============================================================================
JointTrajectoryControllerBase::JointTrajectoryControllerBase()
{
  using hardware_interface::EffortJointInterface;
  using hardware_interface::PositionJointInterface;
  using hardware_interface::VelocityJointInterface;

  mAdapterFactory.registerFactory<PositionJointInterface, JointPositionAdapter>(
      "position");
  mAdapterFactory.registerFactory<VelocityJointInterface, JointVelocityAdapter>(
      "velocity");
  mAdapterFactory.registerFactory<EffortJointInterface, JointEffortAdapter>(
      "effort");
}

//=============================================================================
JointTrajectoryControllerBase::~JointTrajectoryControllerBase() {}

//=============================================================================
bool JointTrajectoryControllerBase::initController(
    hardware_interface::RobotHW* robot, ros::NodeHandle& n)
{
  mNodeHandle.reset(new ros::NodeHandle{n});
  if (!mCancelCurrentTrajectory.is_lock_free()) {
    ROS_ERROR_STREAM(
        "Boolean atomics not lock-free on this system. Cannot guarantee "
        "realtime safety.");
    return false;
  }

  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using hardware_interface::JointStateInterface;

  // load the control type from paramter (position, velocity, effort)
  std::string control_type;
  if (!n.getParam("control_type", control_type)) {
    ROS_ERROR("Failed to load 'control_type' parameter.");
    return false;
  }
  if (control_type != "position" && control_type != "velocity" && control_type != "effort") {
    ROS_ERROR_STREAM("Invalid 'control_type' parameter. Must be 'position', 'velocity', or 'effort', but is " << control_type);
    return false;
  }

  // Build up the list of controlled DOFs.
  const auto jointParameters = loadJointsFromParameter(n, "joints", control_type);
  if (jointParameters.empty()) return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto& param : jointParameters)
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton) return false;

  // Check for zero-mass bodies that will be used incorrectly in calculations
  bool hasZeroMassBody = false;
  for (auto body : mSkeleton->getBodyNodes()) {
    if (body->getMass() <= 0.0) {
      ROS_ERROR_STREAM("Robot link '" << body->getName()
                                      << "' has mass = " << body->getMass());
      hasZeroMassBody = true;
    }
  }
  if (hasZeroMassBody) return false;  // TODO is this actually a problem?

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton =
      getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton) return false;

  mControlledSpace =
      std::make_shared<MetaSkeletonStateSpace>(mControlledSkeleton.get());

  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < mControlledSpace->getDimension(); ++i)
  {
      subspaces.emplace_back(std::make_shared<aikido::statespace::R1>());
  }
  mCompoundSpace
      = std::move(std::make_shared<aikido::statespace::CartesianProduct>(subspaces));

  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface) {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
      new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  // Load goal constraints
  mGoalConstraints = loadGoalConstraintsFromParameter(n, jointParameters);

  // Create adaptors to provide a uniform interface to different types.
  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mAdapters.resize(numControlledDofs);

  for (size_t idof = 0; idof < numControlledDofs; ++idof) {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter) return false;

    // Initialize the adapter using parameters stored on the parameter server.
    ros::NodeHandle adapterNodeHandle = createDefaultAdapterNodeHandle(n, dof);
    if (!adapter->initialize(adapterNodeHandle)) return false;

    mAdapters[idof] = std::move(adapter);
  }

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mDesiredAcceleration.resize(numControlledDofs);
  mDesiredEffort.resize(numControlledDofs);

  // Start the action server. This must be last.
  using std::placeholders::_1;
  mActionServer.reset(new actionlib::ActionServer<Action>{
      n, "follow_joint_trajectory",
      std::bind(&JointTrajectoryControllerBase::goalCallback, this, _1),
      std::bind(&JointTrajectoryControllerBase::cancelCallback, this, _1),
      false});
  mActionServer->start();

  mNonRealtimeTimer = n.createTimer(
      ros::Duration(0.02), &JointTrajectoryControllerBase::nonRealtimeCallback,
      this, false, false);

  return true;
}

//=============================================================================
void JointTrajectoryControllerBase::startController(const ros::Time& time)
{
  mSkeletonUpdater->update();

  // Hold the current position.
  mDesiredPosition = mControlledSkeleton->getPositions();
  mDesiredVelocity.setZero();
  mDesiredAcceleration.setZero();

  ROS_DEBUG_STREAM(
      "Initialized desired position: " << mDesiredPosition.transpose());
  ROS_DEBUG_STREAM(
      "Initialized desired velocity: " << mDesiredVelocity.transpose());
  ROS_DEBUG_STREAM(
      "Initialized desired acceleration: " << mDesiredAcceleration.transpose());

  // Reset any internal state in the adapters (e.g. integral windup).
  for (const auto& adapter : mAdapters) adapter->reset();

  ROS_DEBUG("Reset joint adapters.");

  // Just dump all the previous pending requests. It's too dangerous to go
  // through and abort all of them in the realtime thread.
  mNewTrajectoryRequests.clear();
  mCancelRequests.clear();

  mCurrentTrajectory.set(nullptr);
  mCancelCurrentTrajectory.store(false);
  mAbortCurrentTrajectory.store(false);

  mNonRealtimeTimer.start();
}

//=============================================================================
void JointTrajectoryControllerBase::stopController(const ros::Time& time)
{
  mNonRealtimeTimer.stop();
}

//=============================================================================
void JointTrajectoryControllerBase::updateStep(const ros::Time& time,
                                               const ros::Duration& period)
{

  auto mDesiredState = mCompoundSpace->createState();

  std::shared_ptr<TrajectoryContext> context;
  mCurrentTrajectory.get(context);

  // Update the state of the Skeleton.
  mSkeletonUpdater->update();
  mActualPosition = mControlledSkeleton->getPositions();
  mActualVelocity = mControlledSkeleton->getVelocities();
  mActualEffort = mControlledSkeleton->getForces();

  if (context && !context->mCompleted.load())
  {
    const auto& trajectory = context->mTrajectory;
    const auto timeFromStart = std::min((time - context->mStartTime).toSec(),
                                        trajectory->getEndTime());

    // Evaluate the trajectory at the current time.
    trajectory->evaluate(timeFromStart, mDesiredState);
    mCompoundSpace->logMap(mDesiredState, mDesiredPosition);

    // Apply offset
    mDesiredPosition -= mCurrentTrajectoryOffset;

    trajectory->evaluateDerivative(timeFromStart, 1, mDesiredVelocity);
    trajectory->evaluateDerivative(timeFromStart, 2, mDesiredAcceleration);

    ROS_INFO_STREAM("Desired Velocity: " << mDesiredVelocity.transpose());

    // TODO: Check path constraints.

    // Check goal constraints.
    bool goalConstraintsSatisfied = true;
    for (const auto& dof : mControlledSkeleton->getDofs())
    {
      auto goalIt = mGoalConstraints.find(dof->getName());
      if (goalIt != mGoalConstraints.end())
      {
        std::size_t index = mControlledSkeleton->getIndexOf(dof);
        auto diff = std::abs(mDesiredPosition[index] - mActualPosition[index]);

        // Check for SO2:
        auto jointSpace = mControlledSpace->getJointSpace(index);
        auto r1Joint = std::dynamic_pointer_cast<const R1Joint>(jointSpace);
        if(!r1Joint) {
          if(diff > M_PI) {
            diff = 2 * M_PI - diff;
          }
        }

        if (diff > (*goalIt).second)
        {
          goalConstraintsSatisfied = false;
          break;
        }
      }
    }

    std::string stopReason;
    bool shouldStopExec = shouldStopExecution(stopReason);

    // Terminate the current trajectory.
    if (timeFromStart >= trajectory->getDuration() && goalConstraintsSatisfied)
    {
      context->mCompleted.store(true);
    }
    else if (shouldStopExec || mCancelCurrentTrajectory.load())
    {
      // TODO: if there is no other work that needs done here, we can get rid of
      // the cancel atomic_bool. We do not make the desired velocity and acceleration
      // zero since the trajectory can potentially have been appended with another.
      context->mCompleted.store(true);

      if (shouldStopExec)
      {
        mDesiredVelocity.fill(0.0);
        mDesiredAcceleration.fill(0.0);

        mAbortCurrentTrajectory.store(true);
        mAbortReason = stopReason;
      }
    }
  }

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
    // Check for SO2
    auto actualPos = mActualPosition[idof];
    auto desiredPos = mDesiredPosition[idof];
    auto jointSpace = mControlledSpace->getJointSpace(idof);
    auto r1Joint = std::dynamic_pointer_cast<const R1Joint>(jointSpace);
    if(!r1Joint) {
      if(desiredPos - actualPos > M_PI) {
        actualPos += 2*M_PI;
      } else if(desiredPos - actualPos < -M_PI) {
        actualPos -= 2*M_PI;
      }
    }

    // Call Adapter
    mAdapters[idof]->update(time, period, actualPos,
                            desiredPos, mActualVelocity[idof],
                            mDesiredVelocity[idof], mDesiredEffort[idof]);
  }
}

//=============================================================================
void JointTrajectoryControllerBase::goalCallback(GoalHandle goalHandle)
{
  const auto goal = goalHandle.getGoal();
  ROS_INFO_STREAM("Received trajectory '"
                  << goalHandle.getGoalID().id << "' with "
                  << goal->trajectory.points.size() << " waypoints.");

  if (!shouldAcceptRequests()) {
    Result result;
    result.error_code = Result::INVALID_GOAL;
    result.error_string = "Controller not running.";
    goalHandle.setRejected(result);
    return;
  }

  // Convert the JointTrajectory message to a format that we can execute.
  std::shared_ptr<aikido::trajectory::Spline> trajectory;
  try {
    trajectory = aikido::control::ros::toSplineJointTrajectory(
      mControlledSpace, goal->trajectory, mControlledSkeleton->getPositions());
  } catch (const std::runtime_error& e) {
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
  if (!specifiedStartTime.isZero()) {
    startTime = specifiedStartTime;
  } else {
    startTime = now;
    ROS_INFO(
        "Trajectory does not have an explicit start time, so"
        " we assume that it will start immediately.");
  }

  if (startTime < now) {
    std::stringstream message;
    message << "Trajectory starts in the past: " << specifiedStartTime << " < "
            << now << ".";

    Result result;
    result.error_code = Result::OLD_HEADER_TIMESTAMP;
    result.error_string = message.str();
    goalHandle.setRejected(result);

    ROS_ERROR_STREAM("Rejected trajectory: " << message.str());
    return;
  }

  ROS_INFO_STREAM("Trajectory will start at time " << startTime);

  // Setup the new trajectory.
  const auto newContext = std::make_shared<TrajectoryContext>();
  newContext->mStartTime = startTime;
  newContext->mTrajectory = trajectory;
  newContext->mGoalHandle = goalHandle;

  // Evaluate the trajectory at the current time.
  auto initialTrajectoryState = mControlledSpace->createState();
  Eigen::VectorXd initialTrajectoryPosition(mControlledSpace->getDimension());
  trajectory->evaluate(0, initialTrajectoryState);
  mCompoundSpace->logMap(initialTrajectoryState, initialTrajectoryPosition);

  Eigen::VectorXd actualSkeletonPosition(mControlledSpace->getDimension());
  actualSkeletonPosition = mControlledSkeleton->getPositions();

  Eigen::VectorXd offset(mControlledSpace->getDimension());
  offset = initialTrajectoryPosition - actualSkeletonPosition;

  // The offset is a strict multiple of 2*M_PI to disallow any arbitrary
  // jumps in the execution i.e. if the next trajectory is offset from 2pi
  // in its start state, we still expect a smoother transition.
  auto multiplier = offset / (2*M_PI);
  for (int i = 0; i < offset.size(); ++i)
  {
    offset(i) = round(multiplier(i))*2*M_PI;
  }
  mCurrentTrajectoryOffset = offset;

  ROS_WARN_STREAM("Offset " << offset.transpose());

  newContext->mGoalHandle.setAccepted();
  {  // enter critical section
    std::lock_guard<std::mutex> newTrajectoryLock{mNewTrajectoryRequestsMutex};
    mNewTrajectoryRequests.push_back(newContext);
  }  // exit critical section
}

//=============================================================================
void JointTrajectoryControllerBase::cancelCallback(GoalHandle goalHandle)
{
  ROS_INFO_STREAM("Requesting cancelation of trajectory '"
                  << goalHandle.getGoalID().id << "'.");
  if (!shouldAcceptRequests()) {
    Result result;
    result.error_code = Result::INVALID_GOAL;
    result.error_string = "Controller not running.";
    goalHandle.setRejected(result);
  } else {
    goalHandle.setAccepted();
    std::lock_guard<std::mutex> cancelTrajectoryLock{mCancelRequestsMutex};
    mCancelRequests.push_back(goalHandle);
  }
}

//=============================================================================
void JointTrajectoryControllerBase::nonRealtimeCallback(
    const ros::TimerEvent& event)
{
  TrajectoryContextPtr latestTrajRequest;

  {  // enter critical section
    std::lock(mNewTrajectoryRequestsMutex, mCancelRequestsMutex);
    std::lock_guard<std::mutex> newTrajectoryLock{mNewTrajectoryRequestsMutex,
                                                  std::adopt_lock};
    std::lock_guard<std::mutex> cancelRequestsLock{mCancelRequestsMutex,
                                                   std::adopt_lock};

    // process and delete pending cancel requests
    for (auto cancelItr = mCancelRequests.begin();
         cancelItr != mCancelRequests.end();) {
      if (processCancelRequest(*cancelItr)) {
        cancelItr = mCancelRequests.erase(cancelItr);
      } else {
        ++cancelItr;
      }
    }

    // find latest new trajectory requested and cancel other unstarted
    // trajectories
    if (!mNextTrajectory && !mNewTrajectoryRequests.empty()) {
      mNextTrajectory = mNewTrajectoryRequests.front();
      mNewTrajectoryRequests.pop_front();
    }
  }  // exit critical section

  TrajectoryContextPtr currentTraj;
  mCurrentTrajectory.get(currentTraj);
  if (currentTraj) {
    // if completed...
    if (currentTraj->mCompleted.load()) {
      // if completed due to cancelation inform caller
      if (mCancelCurrentTrajectory.load()) {
        ROS_INFO_STREAM("Canceled trajectory '"
                        << currentTraj->mGoalHandle.getGoalID().id << "'.");
        currentTraj->mGoalHandle.setCanceled();
      }
      // if completed due to being aborted
      else if (mAbortCurrentTrajectory.load()) {
        ROS_INFO_STREAM("Aborted trajectory '"
                        << currentTraj->mGoalHandle.getGoalID().id << "'. Reason: " << mAbortReason);
        currentTraj->mGoalHandle.setAborted(Result(), mAbortReason);
      }
      // if completed due to finishing trajectory set success and reset
      else {
        ROS_INFO_STREAM("Trajectory '"
                        << currentTraj->mGoalHandle.getGoalID().id
                        << "' completed successfully.");
        currentTraj->mGoalHandle.setSucceeded();
      }
      // reset trajectory upon completion
      mCancelCurrentTrajectory.store(false);
      mAbortCurrentTrajectory.store(false);
      mCurrentTrajectory.set(
          mNextTrajectory);  // either sets to nullptr or next
                             // trajectory if available
      mNextTrajectory.reset();
    }
  }
  // if no active trajectory directly start next trajectory
  else {
    mCancelCurrentTrajectory.store(false);
    mCurrentTrajectory.set(mNextTrajectory);  // either sets to nullptr or next
                                              // trajectory if available
    mNextTrajectory.reset();
  }
  publishFeedback(event.current_real);
}

//=============================================================================
bool JointTrajectoryControllerBase::processCancelRequest(GoalHandle& ghToCancel)
{
  std::string ghIdToCancel = ghToCancel.getGoalID().id;
  // check if should cancel active trajectory
  TrajectoryContextPtr existingTraj;
  mCurrentTrajectory.get(existingTraj);
  if (existingTraj
      && ghIdToCancel == existingTraj->mGoalHandle.getGoalID().id) {
    mCancelCurrentTrajectory.store(true);
    return true;
  } else {
    // otherwise cancel pending trajectories
    for (auto itr = mNewTrajectoryRequests.begin();
         itr != mNewTrajectoryRequests.end(); ++itr) {
      if (ghIdToCancel == (*itr)->mGoalHandle.getGoalID().id) {
        (*itr)->mGoalHandle.setCanceled();
        ROS_INFO_STREAM("Canceled trajectory '" << ghIdToCancel << "'.");
        mNewTrajectoryRequests.erase(itr);
        return true;
      }
    }
    // didn't find request to cancel
    return false;
  }
}

//=============================================================================
void JointTrajectoryControllerBase::publishFeedback(
    const ros::Time& currentTime)
{
  std::shared_ptr<TrajectoryContext> context;
  mCurrentTrajectory.get(context);

  if (context) {
    const ros::Duration timeFromStart{currentTime - context->mStartTime};

    Feedback feedback;
    feedback.header.stamp = currentTime;  // TODO: Use control loop time.

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

//=============================================================================
bool JointTrajectoryControllerBase::shouldStopExecution(std::string& message) { return false; }
}  // namespace rewd_controllers
