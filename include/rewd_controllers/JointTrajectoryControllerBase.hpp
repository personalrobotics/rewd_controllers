#ifndef REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLERBASE_HPP_
#define REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLERBASE_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include <actionlib/server/action_server.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <dart/dynamics/dynamics.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <rewd_controllers/helpers.hpp>
#include <ros/node_handle.h>

namespace rewd_controllers
{
class JointTrajectoryControllerBase
{
protected:
  using Action = control_msgs::FollowJointTrajectoryAction;
  using ActionServer = actionlib::ActionServer<Action>;
  using GoalHandle = ActionServer::GoalHandle;

  using Feedback = control_msgs::FollowJointTrajectoryFeedback;
  using Result = control_msgs::FollowJointTrajectoryResult;

  JointTrajectoryControllerBase();
  virtual ~JointTrajectoryControllerBase();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool initController(hardware_interface::RobotHW* robot, ros::NodeHandle& n);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void startController(const ros::Time& time);
  void stopController(const ros::Time& time);

  /**
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void updateStep(const ros::Time& time, const ros::Duration& period);

private:
  /** \brief Contains all data needed to execute the currently
   * requested trajectory. Shared between real time and non-real
   * time threads.
   *
   * This structure must be used AS A WHOLE, and reassigned
   * in its entirety. Assigning to individual components
   * is UNSAFE with the exception of the mCompleted atomic.
   */
  struct TrajectoryContext {
    ros::Time mStartTime;
    std::shared_ptr<aikido::trajectory::Trajectory> mTrajectory;
    GoalHandle mGoalHandle;
    std::atomic_bool mCompleted;
  };
  using TrajectoryContextPtr = std::shared_ptr<TrajectoryContext>;

  /** \brief Actionlib callback to accept new Goal.
   */
  void goalCallback(GoalHandle goalHandle);

  /** \brief Actionlib callback to cancel previously accepted goal.
   */
  void cancelCallback(GoalHandle goalHandle);

  /** \brief Called by timer to manage new, canceled, and completed trajectory
   * requests.
   */
  void nonRealtimeCallback(const ros::TimerEvent& event);

  /** \brief Helper function to publish feedback on currently executing
   * trajectory in \ref nonRealtimecallback.
   */
  void publishFeedback(const ros::Time& currentTime);

  /** \brief Helper function used to remove a cancel request from the list of
   * new trajectory requests in \ref nonRealtimecallback.
   *
   * Does not lock, so must only be called from \ref nonRealtimecallback within
   * locked critical section.
   *
   * \returns True if ghToCancel was properly canceled and should be removed
   * from the queue of cancel requests.
   */
  bool processCancelRequest(GoalHandle& ghToCancel);

  std::unique_ptr<ros::NodeHandle> mNodeHandle;
  JointAdapterFactory mAdapterFactory;
  dart::dynamics::SkeletonPtr mSkeleton;
  dart::dynamics::MetaSkeletonPtr mControlledSkeleton;
  std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace>
      mControlledSpace;

  std::unique_ptr<SkeletonJointStateUpdater> mSkeletonUpdater;
  std::vector<std::unique_ptr<JointAdapter>> mAdapters;
  Eigen::VectorXd mDesiredPosition;
  Eigen::VectorXd mDesiredVelocity;
  Eigen::VectorXd mDesiredAcceleration;
  Eigen::VectorXd mDesiredEffort;
  Eigen::VectorXd mActualPosition;
  Eigen::VectorXd mActualVelocity;
  Eigen::VectorXd mActualEffort;

  std::unique_ptr<ActionServer> mActionServer;
  ros::Timer mNonRealtimeTimer;

  std::vector<GoalHandle> mCancelRequests;
  std::mutex mCancelRequestsMutex;
  std::deque<TrajectoryContextPtr> mNewTrajectoryRequests;
  std::mutex mNewTrajectoryRequestsMutex;

  // TODO: It would be better to use std::atomic<std::shared_ptr<T>> here.
  // However, this is not fully implemented in GCC 4.8.4, shipped with Ubuntu
  // 14.04.
  realtime_tools::RealtimeBox<TrajectoryContextPtr> mCurrentTrajectory;
  std::atomic_bool mCancelCurrentTrajectory;

  TrajectoryContextPtr mNextTrajectory;
};

}  // namespace rewd_controllers

#endif  // ifndef REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLERBASE_HPP_
