#ifndef REWD_CONTROLLERS_MOVEUNTILTOUCHTOPICCONTROLLER_HPP_
#define REWD_CONTROLLERS_MOVEUNTILTOUCHTOPICCONTROLLER_HPP_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <mutex>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <pr_control_msgs/TriggerAction.h>
#include <pr_hardware_interfaces/TriggerableInterface.h>
#include <rewd_controllers/JointTrajectoryControllerBase.hpp>
#include <rewd_controllers/MultiInterfaceController.hpp>

namespace rewd_controllers {

/// Uses a standard JointTrajectoryControllerBase and aborts the trajectory if
/// the forces or torques are too big.
/// Unlike MoveUntilTouchController, this controller gets its F/T data not
/// via a hardware interface but rather via a ros topic.
/// The other difference is, that taring is instigated through an action client
/// instead of a hardware interface.
class MoveUntilTouchTopicController final
    : public MultiInterfaceController<
          hardware_interface::PositionJointInterface,
          hardware_interface::VelocityJointInterface,
          hardware_interface::EffortJointInterface,
          hardware_interface::JointStateInterface>,
      public JointTrajectoryControllerBase {

public:
  MoveUntilTouchTopicController();
  ~MoveUntilTouchTopicController();

  // Documentation inherited
  bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update. It triggers a Tare on the controlled
   * force/torque sensor hardware.
   *
   * \param time The current time
   */
  void starting(const ros::Time &time) override;

  // Documentation inherited
  void stopping(const ros::Time &time) override;

  // Documentation inherited.
  void update(const ros::Time &time, const ros::Duration &period) override;

protected:
  /** \brief The JointTrajectoryControllerBase should accept new trajectories
   * and cancel requests when this controller is started.
   *
   * \returns true when isRunning() is true;
   */
  bool shouldAcceptRequests() override;

  /**
   * \brief Called from the real-time thread every control cycle, this method
   * reads the current force/torque sensor state and compares with
   * tolerances specified by the set_forcetorque_threshold service.
   *
   * \returns True if current wrench exceeds force/torque threshold. If a
   * threshold is set to `0.0` (the default), it is ignored.
   */
  bool shouldStopExecution(std::string &message) override;

private:
  // \brief Force-Torque Thresholding Server
  std::shared_ptr<FTThresholdServer> mFTThresholdServer;
  
};

} // namespace rewd_controllers

#endif // REWD_CONTROLLERS_MOVEUNTILTOUCHTOPICCONTROLLER_HPP_
