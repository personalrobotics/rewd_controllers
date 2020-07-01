#ifndef REWD_CONTROLLERS_MOVEUNTILTOUCHCARTVELOCITY_HPP_
#define REWD_CONTROLLERS_MOVEUNTILTOUCHCARTVELOCITY_HPP_

#include "helpers.hpp"
#include <actionlib/server/action_server.h>
#include <atomic>
#include <controller_interface/multi_interface_controller.h>
#include <dart/dynamics/dynamics.hpp>
#include <memory>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>

#include <pr_control_msgs/SetCartesianVelocityAction.h>
#include <pr_hardware_interfaces/CartesianVelocityInterface.h>

#include <rewd_controllers/FTThresholdServer.hpp>

namespace rewd_controllers {
class MoveUntilTouchCartVelocityController
    : public controller_interface::MultiInterfaceController<
          pr_hardware_interfaces::CartesianVelocityInterface,
          hardware_interface::JointStateInterface,
          hardware_interface::JointModeInterface> {
public:
  MoveUntilTouchCartVelocityController();
  virtual ~MoveUntilTouchCartVelocityController();

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
  bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time &time, const ros::Duration &period);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;

protected:
  using Action = pr_control_msgs::SetCartesianVelocityAction;
  using ActionServer = actionlib::ActionServer<Action>;
  using GoalHandle = ActionServer::GoalHandle;

  using Result = pr_control_msgs::SetCartesianVelocityResult;

  using JointModes = hardware_interface::JointCommandModes;

private:
  // \brief Force-Torque Thresholding Server
  std::shared_ptr<FTThresholdServer> mFTThresholdServer;
  bool mUseFT;
  
  /** \brief Actionlib callback to accept new Goal.
   */
  void goalCallback(GoalHandle goalHandle);

  /** \brief Actionlib callback to cancel previously accepted goal.
   */
  void cancelCallback(GoalHandle goalHandle);

  /** \brief Contains all data needed to execute the currently
   * requested velocity. Shared between real time and non-real
   * time threads.
   *
   * This structure must be used AS A WHOLE, and reassigned
   * in its entirety. Assigning to individual components
   * is UNSAFE with the exception of the mCompleted atomic.
   */
  struct CartVelContext {
    ros::Time mStartTime;
    ros::Duration mDuration;
    Eigen::VectorXd mDesiredVelocity;
    GoalHandle mGoalHandle;
    std::atomic_bool mCompleted;
  };
  using CartVelContextPtr = std::shared_ptr<CartVelContext>;

  // For position feedback:
  dart::dynamics::SkeletonPtr mSkeleton;
  std::unique_ptr<SkeletonJointStateUpdater> mSkeletonUpdater;

  // Action server variables
  std::unique_ptr<ActionServer> mActionServer;

  // TODO: It would be better to use std::atomic<std::shared_ptr<T>> here.
  // However, this will not be implemented until C++20.
  realtime_tools::RealtimeBox<CartVelContextPtr> mCurrentCartVel;

  // For communication with robot
  pr_hardware_interfaces::CartesianVelocityHandle mCartVelHandle;
  hardware_interface::JointModeHandle mJointModeHandle;
  JointModes lastMode;
};

} // namespace rewd_controllers

#endif
