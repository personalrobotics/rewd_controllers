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

namespace rewd_controllers
{

class JointGroupPositionController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  // struct Commands
  // {
  //   double position_; // Last commanded position
  //   double velocity_; // Last commanded velocity
  //   bool has_velocity_; // false if no velocity command has been specified
  // };

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
  void setCommand(const sensor_msgs::JointStateConstPtr& joint_state);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  // void setCommand(double pos_target, double vel_target);

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

  /**
   * \brief Get the PID parameters
   */
  // void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /**
   * \brief Print debug info to console
   */
  // void printDebug();

  /**
   * \brief Get the PID parameters
   */
  // void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

  /**
   * \brief Get the name of the joint this controller uses
   */
  // std::vector<std::string> getJointNames();

  /**
   * \brief Get the current position of the joint
   * \return current position
   */
  // double getPosition();

  std::map<std::string, hardware_interface::JointHandle> joints_;
  std::map<std::string, boost::shared_ptr<const urdf::Joint> > joint_urdfs_;
  realtime_tools::RealtimeBuffer<sensor_msgs::JointStateConstPtr> command_buffer;
  sensor_msgs::JointStateConstPtr joint_state_command;
  // realtime_tools::RealtimeBuffer<std::map<std::string,Commands> > commands_;
  // std::map<std::string,Commands> command_structs_; // pre-allocated memory that is re-used to set the realtime buffer


private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */
  kdl_extension::KdlTreeId kdl_tree_id;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  // void setCommandCB(const sensor_msgs::JointStateConstPtr& msg);

  /**
   * \brief Check that the command is within the hard limits of the joint. Checks for joint
   *        type first. Sets command to limit if out of bounds.
   * \param command - the input to test
   */
  // void enforceJointLimits(double &command);

};

} // namespace

#endif
