#ifndef REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLERBASE_HPP_
#define REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLERBASE_HPP_

#include <memory>
#include <actionlib/server/action_server.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory.hpp>
#include <boost/atomic.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <dart/dynamics/dynamics.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <rewd_controllers/helpers.hpp>
#include <ros/node_handle.h>

// Ensure atomic boolean operations are always lock-free (defined to 2)
#if BOOST_ATOMIC_BOOL_LOCK_FREE != 2
#error "Boolean atomics not lock-free on this system."
#endif

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

  /*!
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
   * is UNSAFE.
   */
  struct TrajectoryContext {
    ros::Time mStartTime;
    std::shared_ptr<aikido::trajectory::Trajectory> mTrajectory;
    GoalHandle mGoalHandle;
  };

  void goalCallback(GoalHandle goalHandle);
  void cancelCallback(GoalHandle goalHandle);
  void nonRealtimeCallback(const ros::TimerEvent& event);
  void publishFeedback(const ros::Time& currentTime);

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

  // It would be better to use std::atomic<std::shared_ptr<T>> here. However,
  // this is not fully implemented in GCC 4.8.4, shipped with Ubuntu 14.04.
  realtime_tools::RealtimeBox<std::shared_ptr<TrajectoryContext>>
      mCurrentTrajectory;
  realtime_tools::RealtimeBox<std::shared_ptr<TrajectoryContext>>
      mNextTrajectory;

  // Signals to/from real-time thread
  boost::atomic_bool mTrajectoryFinished;
  boost::atomic_bool mCancelTrajectory;
};

}  // namespace rewd_controllers

#endif  // ifndef REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLERBASE_HPP_
