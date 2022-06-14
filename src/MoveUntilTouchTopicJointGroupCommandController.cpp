#include <rewd_controllers/MoveUntilTouchTopicJointGroupCommandController.hpp>

#include <functional>
#include <pluginlib/class_list_macros.h>

namespace rewd_controllers {
//=============================================================================
MoveUntilTouchTopicJointGroupCommandController::MoveUntilTouchTopicJointGroupCommandController()
    : MultiInterfaceController{true} // allow_optional_interfaces
      ,
      JointGroupCommandControllerBase{} {
  // Do nothing.
}

//=============================================================================
MoveUntilTouchTopicJointGroupCommandController::~MoveUntilTouchTopicJointGroupCommandController() {
  // Do nothing.
}

//=============================================================================
bool MoveUntilTouchTopicJointGroupCommandController::init(hardware_interface::RobotHW *robot,
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

  // load max F/T sensor wait time from parameter server
  int maxDelay = 100;
  if (!nh.getParam("max_ft_delay", maxDelay)) {
    ROS_ERROR("Failed to load 'max_ft_delay' parameter.");
    return false;
  }

  // Init FT Threshold Server
  mFTThresholdServer.reset(new FTThresholdServer{
      nh, ft_wrench_name, ft_tare_name, forceLimit, torqueLimit, maxDelay});

  // initialize base trajectory controller
  return initController(robot, nh);
}

//=============================================================================
void MoveUntilTouchTopicJointGroupCommandController::starting(const ros::Time &time) {
  // start base trajectory controller
  startController(time);

  // start FTThresholdServer
  mFTThresholdServer->start();
}

//=============================================================================
void MoveUntilTouchTopicJointGroupCommandController::stopping(const ros::Time &time) {
  // stop base trajectory controller
  stopController(time);

  // stop FTThresholdServer
  mFTThresholdServer->stop();
}

//=============================================================================
void MoveUntilTouchTopicJointGroupCommandController::update(const ros::Time &time,
                                           const ros::Duration &period) {
  // update base trajectory controller
  updateStep(time, period);
}

//=============================================================================
bool MoveUntilTouchTopicJointGroupCommandController::shouldAcceptRequests() {
  return isRunning();
}

//=============================================================================
bool MoveUntilTouchTopicJointGroupCommandController::shouldStopExecution(std::string &message) {
  return mFTThresholdServer->shouldStopExecution(message);
}
} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(rewd_controllers::MoveUntilTouchTopicJointGroupCommandController,
                       controller_interface::ControllerBase)
