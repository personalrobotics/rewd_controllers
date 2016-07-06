#ifndef REWD_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_H
#define REWD_CONTROLLERS__JOINT_GROUP_POSITION_CONTROLLER_H

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <fstream>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <dart/dynamics/dynamics.hpp>
#include <unordered_map>
#include "helpers.hpp"

namespace rewd_controllers {

  class JointGroupPositionController
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
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
    bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n);

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
    JointAdapterFactory mAdapterFactory;
    dart::dynamics::SkeletonPtr mSkeleton;
    dart::dynamics::GroupPtr mControlledSkeleton;
    std::unique_ptr<SkeletonJointStateUpdater> mSkeletonUpdater;
    std::vector<std::unique_ptr<JointAdapter>> mAdapters;

    ros::Subscriber mCommandSubscriber; 

    // Controller
    std::vector<double> joint_state_command_;
    realtime_tools::RealtimeBuffer<std::vector<double> > command_buffer_;
    std::vector<control_toolbox::Pid> joint_pid_controllers_;

    // Model
    std::vector<hardware_interface::JointStateHandle> joint_state_handles_;

    std::vector<hardware_interface::JointHandle> controlled_joint_handles_;
    std::unordered_map<std::string, size_t> controlled_joint_map_;

    // Housekeepting convenience
    size_t number_of_joints_;
  };

} // namespace

#endif
