#ifndef REWD_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_H
#define REWD_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_H

#include <memory>
#include <controller_interface/multi_interface_controller.h>
#include <dart/dynamics/dynamics.hpp>
#include <ros/node_handle.h>
#include "helpers.hpp"

namespace rewd_controllers
{
class GravityCompensationController
    : public controller_interface::
          MultiInterfaceController<hardware_interface::EffortJointInterface,
                                   hardware_interface::JointStateInterface>
{
public:
  GravityCompensationController();
  virtual ~GravityCompensationController();

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
  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

private:
  JointAdapterFactory mAdapterFactory;
  std::vector<std::unique_ptr<JointAdapter>> mAdapters;

  dart::dynamics::SkeletonPtr mSkeleton;
  dart::dynamics::MetaSkeletonPtr mControlledSkeleton;
  std::unique_ptr<SkeletonJointStateUpdater> mSkeletonUpdater;
  std::vector<hardware_interface::JointHandle> mControlledJointHandles;
  Eigen::VectorXd mCalculatedForces;
};

}  // namespace

#endif
