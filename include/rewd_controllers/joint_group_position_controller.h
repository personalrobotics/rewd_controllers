// TODO license/acknowlegement

#ifndef REWD_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_H
#define REWD_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_H

/**
   @class rewd_controllers::JointGroupPositionController
   @brief Joint Group Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "rewd_controllers::JointGroupPositionController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint position to achieve.

   Publishes:

   - @b state (control_msgs::JointControllerState) : // TODO array of?
     Current state of the controller, including pid error and gains.

*/

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <kdl_extension/KdlTreeId.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

namespace rewd_controllers {
  typedef std::vector<double> Command;

  class JointGroupPositionController: public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  public:

    JointGroupPositionController();
    ~JointGroupPositionController();

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
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param command
     */
    void setCommand(const sensor_msgs::JointState& msg);

    /** \brief This is called from within the realtime thread just before the
     * first call to \ref update
     *
     * \param time The current time
     */
    void starting(const ros::Time& time);

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    unsigned int number_of_joints;
    std::vector<std::string> joint_names;
    std::vector<control_toolbox::Pid> joint_pid_controllers;
    std::vector<hardware_interface::JointHandle> joints;
    std::vector<boost::shared_ptr<const urdf::Joint> > joint_urdfs;
    kdl_extension::KdlTreeId kdl_tree_id;
    KDL::Tree kdl_tree;
    KDL::Chain controlled_chain;
    std::string base_link_name, tool_link_name;
    realtime_tools::RealtimeBuffer<Command> command_buffer;
    Command joint_state_command;
    ros::Subscriber command_sub;

    /**
     * \brief Check that the command is within the hard limits of the joint. Checks for joint
     *        type first. Sets command to limit if out of bounds.
     * \param command - the input to test
     */
    // void enforceJointLimits(double &command);

  };

} // namespace

#endif
