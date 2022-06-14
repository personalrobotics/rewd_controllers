#include <rewd_controllers/JointGroupCommandControllerBase.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <aikido/common/Spline.hpp>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::R1Joint;


namespace rewd_controllers {

namespace internal
{
// TODO: create a utils file?
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \t2 indices.
 * If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated mapping vector is
 * <tt>"{2, 1}"</tt>.
 */
template <class T>
inline std::vector<unsigned int> mapping(const T& t1, const T& t2)
{
  typedef unsigned int SizeType;

  // t1 must be a subset of t2
  if (t1.size() > t2.size()) {return std::vector<SizeType>();}

  std::vector<SizeType> mapping_vector(t1.size()); // Return value
  for (typename T::const_iterator t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    typename T::const_iterator t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it) {return std::vector<SizeType>();}
    else
    {
      const SizeType t1_dist = std::distance(t1.begin(), t1_it);
      const SizeType t2_dist = std::distance(t2.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

inline std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace

JointGroupCommandControllerBase::JointGroupCommandControllerBase() {
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

bool JointGroupCommandControllerBase::initController(hardware_interface::RobotHW *robot, ros::NodeHandle &n) {
  mNodeHandle.reset(new ros::NodeHandle{n});

  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using hardware_interface::JointStateInterface;

  // load the control type from paramter (position, velocity, effort)
  std::string control_type;
  if (!n.getParam("control_type", control_type)) {
    ROS_ERROR("Failed to load 'control_type' parameter.");
    return false;
  }
  if (control_type != "position" && control_type != "velocity" &&
      control_type != "effort") {
    ROS_ERROR_STREAM("Invalid 'control_type' parameter. Must be 'position', "
                     "'velocity', or 'effort', but is "
                     << control_type);
    return false;
  }

  // Build up the list of controlled DOFs.
  const auto jointParameters =
      loadJointsFromParameter(n, "joints", control_type);
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto &param : jointParameters)
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton)
    return false;

  // Check for zero-mass bodies that will be used incorrectly in calculations
  bool hasZeroMassBody = false;
  for (auto body : mSkeleton->getBodyNodes()) {
    if (body->getMass() <= 0.0) {
      ROS_ERROR_STREAM("Robot link '" << body->getName()
                                      << "' has mass = " << body->getMass());
      hasZeroMassBody = true;
    }
  }
  if (hasZeroMassBody)
    return false; // TODO is this actually a problem?

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton =
      getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton)
    return false;

  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface) {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
      new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  // Create adaptors to provide a uniform interface to different types.
  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mAdapters.resize(numControlledDofs);

  for (size_t idof = 0; idof < numControlledDofs; ++idof) {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter)
      return false;

    // Initialize the adapter using parameters stored on the parameter server.
    ros::NodeHandle adapterNodeHandle = createDefaultAdapterNodeHandle(n, dof);
    if (!adapter->initialize(adapterNodeHandle))
      return false;

    mAdapters[idof] = std::move(adapter);
  }

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mDesiredAcceleration.resize(numControlledDofs);
  mDesiredEffort.resize(numControlledDofs);

  mName = internal::getLeafNamespace(n);

  // Initialize controlled joints
  std::string param_name = "joints";
  if(!n.getParam(param_name, mJointNames))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }

  // Action status checking update rate
  double action_monitor_rate = 20.0;
  n.getParam("action_monitor_rate", action_monitor_rate);
  mActionMonitorPeriod = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(mName, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

  // Rajat ToDo: Add initial state of controller here

  // ROS API: Subscribed topics
  mSubCommand = n.subscribe<trajectory_msgs::JointTrajectoryPoint>("command", 1, &JointGroupCommandControllerBase::commandCallback, this);

  // Start the action server. This must be last.
  // using std::placeholders::_1; // Rajat check: is this required?

  // ROS API: Action interface
  mActionServer.reset(new ActionServer(n, "joint_group_command",
                                      boost::bind(&JointGroupCommandControllerBase::goalCallback, this, _1),
                                      boost::bind(&JointGroupCommandControllerBase::cancelCallback, this, _1),
                                      false));
  mActionServer->start();

  return true;
}

//=============================================================================
void JointGroupCommandControllerBase::startController(const ros::Time &time) {
  mSkeletonUpdater->update();

  // Rajat doubt: Isn't this useless?
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
  for (const auto &adapter : mAdapters)
    adapter->reset();

  ROS_DEBUG("Reset joint adapters.");

}

//=============================================================================
void JointGroupCommandControllerBase::stopController(const ros::Time &time) {
  preemptActiveGoal(); // Rajat check: Is this required?
}

//=============================================================================
void JointGroupCommandControllerBase::updateStep(const ros::Time &time,
                                               const ros::Duration &period) {

  trajectory_msgs::JointTrajectoryPoint command = *mCommandsBuffer.readFromRT(); // Rajat check: should this be by reference?

  // Update the state of the Skeleton.
  mSkeletonUpdater->update();
  mActualPosition = mControlledSkeleton->getPositions();
  mActualVelocity = mControlledSkeleton->getVelocities();
  mActualEffort = mControlledSkeleton->getForces();

  for (const auto &dof : mControlledSkeleton->getDofs()) 
  {
      std::size_t index = mControlledSkeleton->getIndexOf(dof);
      mDesiredPosition[index] = command.positions[index];
      mDesiredVelocity[index] = command.velocities[index];
      mDesiredAcceleration[index] = command.accelerations[index];
      mDesiredEffort[index] = command.effort[index];
  }

  if(!mForwardController)
  {
    // Compute inverse dynamics torques from the set point and store them in the
    // skeleton. These values may be queried by the adapters below.
    mControlledSkeleton->setPositions(mDesiredPosition);
    mControlledSkeleton->setVelocities(mDesiredVelocity);
    mControlledSkeleton->setAccelerations(mDesiredAcceleration);

    mSkeleton->computeInverseDynamics();
    mDesiredEffort = mControlledSkeleton->getForces();

  }

  // Restore the state of the Skeleton from JointState interfaces. These values
  // may be used by the adapters below.
  mControlledSkeleton->setPositions(mActualPosition);
  mControlledSkeleton->setVelocities(mActualVelocity);

  for (size_t idof = 0; idof < mAdapters.size(); ++idof) {
    // Check for SO2
    auto actualPos = mActualPosition[idof];
    auto desiredPos = mDesiredPosition[idof];

    // Rajat ToDo: Check necessity of below code
    // auto jointSpace = mControlledSpace->getJointSpace(idof);
    // auto r1Joint = std::dynamic_pointer_cast<const R1Joint>(jointSpace);
    // if (!r1Joint) {
    //   desiredPos = std::fmod(desiredPos, 2.0 * M_PI);
    //   actualPos = std::fmod(actualPos, 2.0 * M_PI);
    //   if (desiredPos - actualPos > M_PI) {
    //     desiredPos -= 2 * M_PI;
    //   } else if (desiredPos - actualPos < -M_PI) {
    //     desiredPos += 2 * M_PI;
    //   }
    // }

    // Call Adapter
    try {
      mAdapters[idof]->update(time, period, actualPos, desiredPos,
                            mActualVelocity[idof], mDesiredVelocity[idof],
                            mDesiredEffort[idof]);
    } catch (std::exception& e) {
      // Abort Command
      // Rajat ToDo
    }
  }

  setActionFeedback(time);  // Rajat check: Is this required?
}

void JointGroupCommandControllerBase::preemptActiveGoal()
{
    RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

    // Cancel any goal timeout
    mGoalDurationTimer.stop();

    // Cancels the currently active goal
    if (current_active_goal)
    {
      // Marks the current goal as canceled
      mRTActiveGoal.reset();
      current_active_goal->gh_.setCanceled();
    }
}

void JointGroupCommandControllerBase::commandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
{
  // Preconditions
  if (!shouldAcceptRequests())
  {
    ROS_ERROR_STREAM_NAMED(mName, "Can't accept new commands. Controller is not running.");
    return;
  }

  if (!msg)
  {
    ROS_WARN_STREAM_NAMED(mName, "Received null-pointer message, skipping.");
    return;
  }

  mCommandsBuffer.writeFromNonRT(*msg);
  preemptActiveGoal();
}

void JointGroupCommandControllerBase::goalCallback(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED(mName,"Received new action goal");
  pr_control_msgs::JointGroupCommandResult result;

  // Preconditions
  if (!shouldAcceptRequests())
  {
    result.error_string = "Can't accept new action goals. Controller is not running.";
    ROS_ERROR_STREAM_NAMED(mName, result.error_string);
    result.error_code = pr_control_msgs::JointGroupCommandResult::INVALID_GOAL;
    gh.setRejected(result);
    return;
  }

  // if (gh.getGoal()->joint_names.size() != gh.getGoal()->command.positions.size()) {
  //   result.error_string = "Size of command must match size of joint_names.";
  //   ROS_ERROR_STREAM_NAMED(mName, result.error_string);
  //   result.error_code = pr_control_msgs::JointGroupCommandResult::INVALID_GOAL;
  //   gh.setRejected(result);
  //   return;
  // }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case

  // update new command
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  trajectory_msgs::JointTrajectoryPoint new_command = gh.getGoal()->command;
  rt_goal->preallocated_feedback_->joint_names = mJointNames;
  mCommandsBuffer.writeFromNonRT(new_command);

  // Accept new goal
  preemptActiveGoal();
  gh.setAccepted();
  mRTActiveGoal = rt_goal;

  // Setup goal status checking timer
  mGoalDurationTimer = mNodeHandle->createTimer(mActionMonitorPeriod,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
  mGoalDurationTimer.start();

  // Setup goal timeout
  if (gh.getGoal()->command.time_from_start > ros::Duration()) {
    mGoalDurationTimer = mNodeHandle->createTimer(gh.getGoal()->command.time_from_start,
                                                    &JointGroupCommandControllerBase::timeoutCallback,
                                                    this,
                                                    true);
    mGoalDurationTimer.start();
  }
}

void JointGroupCommandControllerBase::timeoutCallback(const ros::TimerEvent& event)
{
  RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

  // Check that timeout refers to currently active goal (if any)
  if (current_active_goal) {
    ROS_DEBUG_NAMED(mName, "Active action goal reached requested timeout.");

    // Give sub-classes option to update mDefaultCommand
    updateDefaultCommand();
    mCommandsBuffer.writeFromNonRT(mDefaultCommand);

    // Marks the current goal as succeeded
    mRTActiveGoal.reset();
    current_active_goal->gh_.setSucceeded();
  }
}

void JointGroupCommandControllerBase::cancelCallback(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

  // Check that cancel request refers to currently active goal
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    ROS_DEBUG_NAMED(mName, "Canceling active action goal because cancel callback recieved from actionlib.");

    // Give sub-classes option to update mDefaultCommand
    updateDefaultCommand();
    mCommandsBuffer.writeFromNonRT(mDefaultCommand);

    preemptActiveGoal();
  }
}

void JointGroupCommandControllerBase::setActionFeedback(const ros::Time& time)
{
  RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);
  if (!current_active_goal)
  {
    return;
  }

  current_active_goal->preallocated_feedback_->header.stamp = time;
  current_active_goal->preallocated_feedback_->desired = current_active_goal->gh_.getGoal()->command;
  current_active_goal->preallocated_feedback_->actual.positions.clear();
  current_active_goal->preallocated_feedback_->actual.velocities.clear();
  current_active_goal->preallocated_feedback_->actual.effort.clear();
  for (const auto &dof : mControlledSkeleton->getDofs()) 
  {
      std::size_t index = mControlledSkeleton->getIndexOf(dof);
      current_active_goal->preallocated_feedback_->actual.positions.push_back(mActualPosition[index]);
      current_active_goal->preallocated_feedback_->actual.velocities.push_back(mActualVelocity[index]);
      current_active_goal->preallocated_feedback_->actual.effort.push_back(mActualEffort[index]);
  }

  current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );
}

void JointGroupCommandControllerBase::updateDefaultCommand()
{
  mDefaultCommand.positions.clear();
  mDefaultCommand.velocities.clear();
  mDefaultCommand.accelerations.clear();
  mDefaultCommand.effort.clear();

  for (const auto &dof : mControlledSkeleton->getDofs()) 
  {
      std::size_t index = mControlledSkeleton->getIndexOf(dof);
      mDefaultCommand.positions.push_back(mActualPosition[index]);
      mDefaultCommand.velocities.push_back(mActualVelocity[index]);
      // Rajat ToDo: Have default accelarations as well?
      mDefaultCommand.effort.push_back(mActualEffort[index]);
  }
}

//=============================================================================
// Default for virtual function is do nothing. DO NOT EDIT
bool JointGroupCommandControllerBase::shouldStopExecution(std::string &message) {
  return false;
}

} // namespace
