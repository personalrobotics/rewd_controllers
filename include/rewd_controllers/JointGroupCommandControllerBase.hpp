#ifndef REWD_CONTROLLERS_JOINTCOMMANDCONTROLLERBASE_HPP_
#define REWD_CONTROLLERS_JOINTCOMMANDCONTROLLERBASE_HPP_

#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>

// actionlib
#include <actionlib/server/action_server.h>

// ROS messages
#include <pr_control_msgs/JointGroupCommandAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_mode_interface.h>

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <rewd_controllers/helpers.hpp>
#include <ros/node_handle.h>


namespace rewd_controllers {

class JointGroupCommandControllerBase {
public:
  JointGroupCommandControllerBase();
  ~JointGroupCommandControllerBase() { mSubCommand.shutdown();}

  bool initController(hardware_interface::RobotHW *robot, ros::NodeHandle &n);
  void startController(const ros::Time &time);
  void stopController(const ros::Time &time);
  void updateStep(const ros::Time &time, const ros::Duration &period);

  virtual bool shouldAcceptRequests() = 0;
  virtual bool shouldStopExecution(std::string &message);

protected:
  using ActionServer = actionlib::ActionServer<pr_control_msgs::JointGroupCommandAction>;
  using GoalHandle = ActionServer::GoalHandle;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<pr_control_msgs::JointGroupCommandAction>;
  using RealtimeGoalHandlePtr = boost::shared_ptr<RealtimeGoalHandle>;

  realtime_tools::RealtimeBuffer<trajectory_msgs::JointTrajectoryPoint> mCommandsBuffer;
  std::atomic_bool mExecuteDefaultCommand;

  std::string                                    mName;               ///< Controller name.
  std::vector< std::string >                     mJointNames;         ///< Controlled joint names
  RealtimeGoalHandlePtr                          mRTActiveGoal;     ///< Currently active action goal, if any.

  std::unique_ptr<ros::NodeHandle> mNodeHandle;
  JointAdapterFactory mAdapterFactory;
  dart::dynamics::SkeletonPtr mSkeleton;
  dart::dynamics::MetaSkeletonPtr mControlledSkeleton;

  std::unique_ptr<SkeletonJointStateUpdater> mSkeletonUpdater;
  std::vector<std::unique_ptr<JointAdapter>> mAdapters;
  Eigen::VectorXd mDesiredPosition;
  Eigen::VectorXd mDesiredVelocity;
  Eigen::VectorXd mDesiredAcceleration;
  Eigen::VectorXd mDesiredEffort;
  Eigen::VectorXd mActualPosition;
  Eigen::VectorXd mActualVelocity;
  Eigen::VectorXd mActualEffort;

  bool mCompensateEffort = true;
  ExtendedJointPosition* mExtendedJoints;
  Eigen::MatrixXd mJointStiffnessMatrix;

  ros::Subscriber    mSubCommand;
  std::unique_ptr<ActionServer> mActionServer;
  ros::Timer         mGoalHandleTimer;
  ros::Timer         mGoalDurationTimer;
  ros::Duration      mActionMonitorPeriod;

  void goalCallback(GoalHandle gh);
  void cancelCallback(GoalHandle gh);
  void timeoutCallback(const ros::TimerEvent& event);
  void preemptActiveGoal();
  void commandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

private:
  /**
   * @brief Updates the pre-allocated feedback of the current active goal (if any)
   * based on the current state values.
   *
   * @note This function is NOT thread safe but intended to be used in the
   * update-function.
   */
  void setActionFeedback(const ros::Time& time);
};

} // namespace

#endif // ifndef REWD_CONTROLLERS_JOINTCOMMANDCONTROLLERBASE_HPP_