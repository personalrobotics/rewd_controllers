// TODO license/acknowlegement

#ifndef REWD_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_H
#define REWD_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_H

/**
   @class rewd_controllers::GravityCompensationController
   @brief Gravity Compensation Controller

   This class compensates for gravity only using inverse dynamics

   @section ROS ROS interface

   @param type Must be "rewd_controllers::GravityCompensationController"
*/

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <fstream>
#include <ros/node_handle.h>
#include <dart/dynamics/SmartPointer.h>
#include <unordered_map>

namespace rewd_controllers {

  class GravityCompensationController
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                            hardware_interface::JointStateInterface> {
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
    bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n);

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    // Model
    dart::dynamics::SkeletonPtr skeleton_;
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;

    dart::dynamics::GroupPtr controlled_skeleton_;
    std::vector<hardware_interface::JointHandle> controlled_joint_handles_;
    std::unordered_map<std::string, size_t> controlled_joint_map_;

  };

} // namespace

#endif
