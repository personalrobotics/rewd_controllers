#include <rewd_controllers/MoveUntilTouchTopicController.hpp>

#include <functional>
#include <pluginlib/class_list_macros.h>

namespace rewd_controllers {
//=============================================================================
MoveUntilTouchTopicController::MoveUntilTouchTopicController()
    : MultiInterfaceController{true} // allow_optional_interfaces
      ,
      JointTrajectoryControllerBase{} {
  // Do nothing.
}

//=============================================================================
MoveUntilTouchTopicController::~MoveUntilTouchTopicController() {
  // Do nothing.
}

//=============================================================================
bool MoveUntilTouchTopicController::init(hardware_interface::RobotHW *robot,
                                         ros::NodeHandle &nh) {
  // load name of force/torque sensor handle from paramter
  std::string ft_wrench_name;
  if (!nh.getParam("forcetorque_wrench_name", ft_wrench_name)) {
    ROS_ERROR("Failed to load 'forcetorque_wrench_name' parameter.");
    return false;
  }
  // load name of force/torque tare handle from paramter
  std::string ft_tare_name;
  if (!nh.getParam("forcetorque_tare_name", ft_tare_name)) {
    ROS_ERROR("Failed to load 'forcetorque_tare_name' parameter.");
    return false;
  }

  // load force/torque saturation limits from parameter
  double forceLimit = 0.0;
  if (!nh.getParam("sensor_force_limit", forceLimit)) {
    ROS_ERROR("Failed to load 'sensor_force_limit' parameter.");
    return false;
  }
  double torqueLimit = 0.0;
  if (!nh.getParam("sensor_torque_limit", torqueLimit)) {
    ROS_ERROR("Failed to load 'sensor_torque_limit' parameter.");
    return false;
  }
  if (forceLimit < 0) {
    ROS_ERROR("sensor_force_limit must be positive or zero");
    return false;
  }
  if (torqueLimit < 0) {
    ROS_ERROR("sensor_torque_limit must be positive or zero");
    return false;
  }

  // Init FT Threshold Server
  mFTThresholdServer.reset(new FTThresholdServer{nh,
          ft_wrench_name,
          ft_tare_name,
          forceLimit,
          torqueLimit});

  // initialize base trajectory controller
  return initController(robot, nh);
}

//=============================================================================
void MoveUntilTouchTopicController::starting(const ros::Time &time) {
  // start base trajectory controller
  startController(time);
}

//=============================================================================
void MoveUntilTouchTopicController::stopping(const ros::Time &time) {
  // stop base trajectory controller
  stopController(time);
}

//=============================================================================
void MoveUntilTouchTopicController::update(const ros::Time &time,
                                           const ros::Duration &period) {
  // update base trajectory controller
  updateStep(time, period);
}

//=============================================================================
bool MoveUntilTouchTopicController::shouldAcceptRequests() {
  return isRunning();
}

//=============================================================================
bool MoveUntilTouchTopicController::shouldStopExecution(std::string &message) {
  return mFTThresholdServer->shouldStopExecution(message);
}
} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(rewd_controllers::MoveUntilTouchTopicController,
                       controller_interface::ControllerBase)
