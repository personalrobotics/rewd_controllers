#ifndef REWD_CONTROLLERS_JOINTGROUPPOSITIONCONTROLLER_HPP_
#define REWD_CONTROLLERS_JOINTGROUPPOSITIONCONTROLLER_HPP_

#include "helpers.hpp"
#include "JointAdapterFactory.hpp"
#include <controller_interface/multi_interface_controller.h>
#include <dart/dynamics/dynamics.hpp>
#include <memory>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

namespace rewd_controllers {

class JointGroupPositionController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::PositionJointInterface,
          hardware_interface::VelocityJointInterface,
          hardware_interface::EffortJointInterface,
          hardware_interface::JointStateInterface> {
public:
  JointGroupPositionController();
  virtual ~JointGroupPositionController();

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
  bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time &time) override;

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time &time, const ros::Duration &period) override;

  /*!
   * \brief Give set position of the joint for next update: revolute (angle)
   * and prismatic (position)
   *
   * \param command
   */
  void setCommand(const sensor_msgs::JointState &msg);

private:
  JointAdapterFactory mAdapterFactory;
  dart::dynamics::SkeletonPtr mSkeleton;
  dart::dynamics::MetaSkeletonPtr mControlledSkeleton;
  std::unique_ptr<SkeletonJointStateUpdater> mSkeletonUpdater;
  std::vector<std::unique_ptr<JointAdapter>> mAdapters;

  realtime_tools::RealtimeBuffer<Eigen::VectorXd> mDesiredPositionBuffer;
  Eigen::VectorXd mDesiredPosition;
  ros::Subscriber mCommandSubscriber;
};

} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_JOINTGROUPPOSITIONCONTROLLER_HPP_
