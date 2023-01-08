#include <rewd_controllers/TaskSpaceCompliantController.hpp>

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

#include <pluginlib/class_list_macros.h>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::R1Joint;


namespace rewd_controllers {

namespace internal
{

inline std::string getLeafNamespace(const ros::NodeHandle& nh)
{
  const std::string complete_ns = nh.getNamespace();
  std::size_t id   = complete_ns.find_last_of("/");
  return complete_ns.substr(id + 1);
}

} // namespace

TaskSpaceCompliantController::TaskSpaceCompliantController() 
    : MultiInterfaceController(true) // allow_optional_interfaces
{}

//=============================================================================
TaskSpaceCompliantController::~TaskSpaceCompliantController() {}

bool TaskSpaceCompliantController::init(hardware_interface::RobotHW *robot, ros::NodeHandle &n) {

  using hardware_interface::EffortJointInterface;
  using hardware_interface::JointStateInterface;

  mNodeHandle.reset(new ros::NodeHandle{n});

  // Build up the list of controlled DOFs.
  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto &param : jointParameters) {
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

    if (param.mType != "effort") {
      ROS_ERROR_STREAM("Joint '"
                       << param.mName
                       << "' is not effort-controlled and cannot be "
                          "used in a gravity compensation controller");
      return false;
    }
  }

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton)
    return false;

  // // Check for zero-mass bodies that will be used incorrectly in calculations
  // bool hasZeroMassBody = false;
  // for (auto body : mSkeleton->getBodyNodes()) {
  //   if (body->getMass() <= 0.0) {
  //     ROS_ERROR_STREAM("Robot link '" << body->getName()
  //                                     << "' has mass = " << body->getMass());
  //     hasZeroMassBody = true;
  //   }
  // }
  // if (hasZeroMassBody)
  //   return false; // TODO is this actually a problem?

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

  const auto numControlledDofs = mControlledSkeleton->getNumDofs();

  ROS_INFO_STREAM("numControlledDofs: " << (int)numControlledDofs);

  mControlledJointHandles.resize(numControlledDofs);

  const auto effortJointInterface = robot->get<EffortJointInterface>();
  if (!effortJointInterface) {
    ROS_ERROR("Unable to get EffortJointInterface from RobotHW instance.");
    return false;
  }

  for (size_t idof = 0; idof < numControlledDofs; ++idof) {
    const auto dofName = mControlledSkeleton->getDof(idof)->getName();
    try {
      auto handle = effortJointInterface->getHandle(dofName);
      mControlledJointHandles[idof] = handle;
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM(
          "Unable to get interface of type 'EffortJointInterface' for joint '"
          << dofName << "'.");
      return false;
    }
  }

  ROS_INFO_STREAM("1");

  mExtendedJoints = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);
  mExtendedJointsGravity = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);

  ROS_INFO_STREAM("2");

  mCount = 0;  

  mJointStiffnessMatrix.resize(numControlledDofs, numControlledDofs);
  mJointStiffnessMatrix.setZero();
  mJointStiffnessMatrix.diagonal() << 8000,8000,8000,7000,7000,7000;

    ROS_INFO_STREAM("3");
  
  mRotorInertiaMatrix.resize(numControlledDofs, numControlledDofs);
  mRotorInertiaMatrix.setZero();
  mRotorInertiaMatrix.diagonal() << 0.4, 0.4, 0.4, 0.2, 0.2, 0.2;

  mFrictionL.resize(numControlledDofs, numControlledDofs);
  mFrictionL.setZero();
  mFrictionL.diagonal() << 160, 160, 160, 100, 100, 100;

  mFrictionLp.resize(numControlledDofs, numControlledDofs);
  mFrictionLp.setZero();
  mFrictionLp.diagonal() << 10, 10, 10, 7.5, 7.5, 7.5;

  mJointKMatrix.resize(numControlledDofs, numControlledDofs);
  mJointKMatrix.setZero();
  mJointKMatrix.diagonal() << 80,80,80,60,60,60;

  mJointDMatrix.resize(numControlledDofs, numControlledDofs);
  mJointDMatrix.setZero();
  mJointDMatrix.diagonal() << 8,8,8,6,6,6;


    ROS_INFO_STREAM("4");

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mZeros.resize(numControlledDofs);
  mZeros.setZero();

    ROS_INFO_STREAM("5");

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
  mSubCommand = n.subscribe<trajectory_msgs::JointTrajectoryPoint>("command", 1, &TaskSpaceCompliantController::commandCallback, this);

  // Start the action server. This must be last.
  // using std::placeholders::_1; // Rajat check: is this required?

  // ROS API: Action interface
  mActionServer.reset(new ActionServer(n, "joint_group_command",
                                      boost::bind(&TaskSpaceCompliantController::goalCallback, this, _1),
                                      boost::bind(&TaskSpaceCompliantController::cancelCallback, this, _1),
                                      false));
  mActionServer->start();

  ROS_INFO("TaskSpaceCompliantController initialized successfully");
  return true;
}

//=============================================================================
void TaskSpaceCompliantController::starting(const ros::Time &time) {

  mExecuteDefaultCommand = true;

  ROS_DEBUG_STREAM(
      "Initialized desired position: " << mDesiredPosition.transpose());
  ROS_DEBUG_STREAM(
      "Initialized desired velocity: " << mDesiredVelocity.transpose());

  ROS_DEBUG("Reset PID.");

}

//=============================================================================
void TaskSpaceCompliantController::stopping(const ros::Time &time) {
  preemptActiveGoal(); // Rajat check: Is this required?
}

//=============================================================================
void TaskSpaceCompliantController::update(const ros::Time &time,
                                               const ros::Duration &period) {

  // Update the state of the Skeleton.
  mSkeletonUpdater->update();
  mActualPosition = mControlledSkeleton->getPositions();
  mActualVelocity = mControlledSkeleton->getVelocities();
  mActualEffort = mControlledSkeleton->getForces();

  std::string stopReason;
  bool shouldStopExec = shouldStopExecution(stopReason);

  if(shouldStopExec)
  {
    std::cout<<"Controller halted due to: "<<stopReason<<std::endl;
    preemptActiveGoal();
    mExecuteDefaultCommand = true;
  }
  
  if(shouldStopExec || mExecuteDefaultCommand.load())
  {
    std::cout<<"1. Updating desired position ..."<<std::endl;
    mDesiredPosition = mActualPosition;
    mDesiredVelocity.setZero();
  }
  else
  {
    // std::cout<<"Reading from RT Buffer... "<<std::endl;
    trajectory_msgs::JointTrajectoryPoint command = *mCommandsBuffer.readFromRT(); // Rajat check: should this be by reference?
    // std::cout<<"... Read. :| "<<std::endl;
  
    // std::cout<<"Efforts Size: "<<command.effort.size()<<std::endl;
    // std::cout<<"Positions Size: "<<command.positions.size()<<std::endl;
    // std::cout<<"Velocities Size: "<<command.velocities.size()<<std::endl;
    // std::cout<<"Accelerations Size: "<<command.accelerations.size()<<std::endl;

    for (const auto &dof : mControlledSkeleton->getDofs()) 
    {
      // Rajat ToDo: Find better method
      std::size_t index = mControlledSkeleton->getIndexOf(dof);
      mDesiredPosition[index] = (command.positions.size() == 0) ? 0.0 : command.positions[index];
      mDesiredVelocity[index] = (command.velocities.size() == 0) ? 0.0 : command.velocities[index];
    }
    // std::cout<<"Out. :) "<<std::endl;
  }

  {

    mExtendedJointsGravity->is_initialized = false;
    mExtendedJointsGravity->initializeExtendedJointPosition(mDesiredPosition);
    mExtendedJointsGravity->estimateExtendedJoint(mExtendedJointsGravity->mLastDesiredPosition);
    mActualTheta = mExtendedJointsGravity->getExtendedJoint();

    mControlledSkeleton->setPositions(mActualTheta);
    mControlledSkeleton->setVelocities(mZeros);
    mControlledSkeleton->setAccelerations(mZeros); 
    mSkeleton->computeInverseDynamics();
    mGravity = mControlledSkeleton->getGravityForces();

    // Restore the state of the Skeleton from JointState interfaces. These values
    // may be used by the adapters below.
    mControlledSkeleton->setPositions(mActualPosition);
    mControlledSkeleton->setVelocities(mActualVelocity);

  }

  if (!mExtendedJoints->is_initialized){
    std::cout<<"2. Updating desired position ..."<<std::endl;
    mLastDesiredPosition = mDesiredPosition;
    mExtendedJoints->initializeExtendedJointPosition(mDesiredPosition);
    mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
    mNominalThetaPrev = mExtendedJoints->getExtendedJoint();
    mNominalThetaDotPrev = mActualVelocity;
    mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
    mTrueDesiredVelocity = mDesiredVelocity;
  }

  if (mDesiredPosition != mLastDesiredPosition && mActualPosition != mDesiredPosition){
    std::cout<<"3. Updating desired position ..."<<std::endl;
    mLastDesiredPosition = mDesiredPosition;
    mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
    mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
    mTrueDesiredVelocity = mDesiredVelocity;
  }
  
  mExtendedJoints->estimateExtendedJoint(mActualPosition);
  mActualTheta = mExtendedJoints->getExtendedJoint();

  mDesiredTheta = mTrueDesiredPosition + mJointStiffnessMatrix.inverse()*mGravity;
  mDesiredThetaDot = mTrueDesiredVelocity;
  
  mTaskEffort = -mJointKMatrix*(mNominalThetaPrev-mDesiredTheta) - mJointDMatrix*(mNominalThetaDotPrev - mDesiredThetaDot) + mGravity;

  double step_time;
  step_time = 0.001;

  mNominalThetaDDot = mRotorInertiaMatrix.inverse()*(mTaskEffort+mActualEffort); // mActualEffort is negative of what is required here
  mNominalThetaDot = mNominalThetaDotPrev + mNominalThetaDDot*step_time;
  mNominalTheta = mNominalThetaPrev + mNominalThetaDot*step_time;

  mNominalThetaPrev = mNominalTheta;
  mNominalThetaDotPrev = mNominalThetaDot;

  mNominalFriction = mRotorInertiaMatrix*mFrictionL*((mNominalThetaDotPrev - mActualVelocity) + mFrictionLp*(mNominalThetaPrev - mActualTheta));

  mDesiredEffort = mTaskEffort + mNominalFriction;

  if(mCount < 2000)
  {
    mDesiredEffort = mGravity;
    mCount++;
    if(mCount%200 == 0)
      std::cout<<"Initializing controller: "<<mCount<<std::endl; 
  }

  std::cout<<"mTaskEffort: "<<mTaskEffort.transpose()<<std::endl;
  std::cout<<"mNominalFriction: "<<mNominalFriction.transpose()<<std::endl;
  std::cout<<"mActualTheta: "<<mActualTheta.transpose()<<std::endl;
  std::cout<<"mActualEffort: "<<mActualEffort.transpose()<<std::endl;

  for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof) 
  {
    auto jointHandle = mControlledJointHandles[idof];
    jointHandle.setCommand(mDesiredEffort[idof]);
  }

  setActionFeedback(time);  // Rajat check: Is this required?
}

void TaskSpaceCompliantController::preemptActiveGoal()
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

void TaskSpaceCompliantController::commandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
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
  mExecuteDefaultCommand = false;
}

void TaskSpaceCompliantController::goalCallback(GoalHandle gh)
{
  std::cout<<"Joint group command controller: Recieved new goal!"<<std::endl;
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
  mExecuteDefaultCommand = false;

  // Setup goal status checking timer
  mGoalHandleTimer = mNodeHandle->createTimer(mActionMonitorPeriod,
                                                    &RealtimeGoalHandle::runNonRealtime,
                                                    rt_goal);
  mGoalHandleTimer.start();

  // Setup goal timeout
  if (gh.getGoal()->command.time_from_start > ros::Duration()) {
    mGoalDurationTimer = mNodeHandle->createTimer(gh.getGoal()->command.time_from_start,
                                                    &TaskSpaceCompliantController::timeoutCallback,
                                                    this,
                                                    true);
    mGoalDurationTimer.start();
  }
}

void TaskSpaceCompliantController::timeoutCallback(const ros::TimerEvent& event)
{
  RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

  // Check that timeout refers to currently active goal (if any)
  if (current_active_goal) {
    ROS_DEBUG_NAMED(mName, "Active action goal reached requested timeout.");

    // Give sub-classes option to update mDefaultCommand
    mExecuteDefaultCommand = true;

    // Marks the current goal as succeeded
    mRTActiveGoal.reset();
    current_active_goal->gh_.setSucceeded();
  }
}

void TaskSpaceCompliantController::cancelCallback(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

  // Check that cancel request refers to currently active goal
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    ROS_DEBUG_NAMED(mName, "Canceling active action goal because cancel callback recieved from actionlib.");

    // Give sub-classes option to update mDefaultCommand
    mExecuteDefaultCommand = true;

    preemptActiveGoal();
  }
}

void TaskSpaceCompliantController::setActionFeedback(const ros::Time& time)
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

//=============================================================================
bool TaskSpaceCompliantController::shouldAcceptRequests() { 
  return isRunning(); 
}

//=============================================================================
// Default for virtual function is do nothing. DO NOT EDIT
bool TaskSpaceCompliantController::shouldStopExecution(std::string &message) {
  return false;
}

} // namespace


//=============================================================================
PLUGINLIB_EXPORT_CLASS(rewd_controllers::TaskSpaceCompliantController,
                       controller_interface::ControllerBase)