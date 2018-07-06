#ifndef REWD_CONTROLLERS_MOVEUNTILTOUCHCONTROLLER_HPP_
#define REWD_CONTROLLERS_MOVEUNTILTOUCHCONTROLLER_HPP_

#include <atomic>
#include <actionlib/server/action_server.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <rewd_controllers/TriggerableInterface.hpp>
#include <rewd_controllers/MultiInterfaceController.hpp>
#include <rewd_controllers/JointTrajectoryControllerBase.hpp>

namespace rewd_controllers
{
class MoveUntilTouchController final
    : public MultiInterfaceController<hardware_interface::
                                          PositionJointInterface,
                                      hardware_interface::
                                          VelocityJointInterface,
                                      hardware_interface::EffortJointInterface,
                                      hardware_interface::JointStateInterface,
                                      hardware_interface::
                                          ForceTorqueSensorInterface,
                                      pr_hardware_interfaces::
                                          TriggerableInterface>,
      public JointTrajectoryControllerBase
{
public:
  MoveUntilTouchController();
  ~MoveUntilTouchController();

  // Documentation inherited
  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update. It triggers a Tare on the controlled
   * force/torque sensor hardware.
   *
   * \param time The current time
   */
  void starting(const ros::Time& time) override;

  // Documentation inherited
  void stopping(const ros::Time& time) override;

  // Documentation inherited.
  void update(const ros::Time& time, const ros::Duration& period) override;

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
   * \param message If the execution should be stopped, this contains reason for stopping the execution.
   * 
   * \returns True if current wrench exceeds force/torque threshold. If a
   * threshold is set to `0.0` (the default), it is ignored.
   */
  bool shouldStopExecution(std::string& message) override;

private:
  using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
  using FTThresholdActionServer = actionlib::ActionServer<SetFTThresholdAction>;
  using FTThresholdGoalHandle = FTThresholdActionServer::GoalHandle;
  using FTThresholdResult = pr_control_msgs::SetForceTorqueThresholdResult;

  hardware_interface::ForceTorqueSensorHandle mForceTorqueHandle;
  pr_hardware_interfaces::TriggerableHandle mTareHandle;
  std::atomic_bool mTaringCompleted;
  double mForceLimit;
  double mTorqueLimit;

  std::unique_ptr<FTThresholdActionServer> mFTThresholdActionServer;
  std::atomic<double> mForceThreshold;
  std::atomic<double> mTorqueThreshold;

  /**
   * \brief Callback for pr_control_msgs::SetForceTorqueThresholdAction.
   */
  void setForceTorqueThreshold(FTThresholdGoalHandle gh);
};

}  // namespace rewd_controllers

#endif  // REWD_CONTROLLERS_MOVEUNTILTOUCHCONTROLLER_HPP_
