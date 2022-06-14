#include <pluginlib/class_list_macros.h>
#include <rewd_controllers/JointGroupCommandController.hpp>

namespace rewd_controllers {
//=============================================================================
JointGroupCommandController::JointGroupCommandController()
    : MultiInterfaceController{true} // allow_optional_interfaces
      ,
      JointGroupCommandControllerBase{} {}

//=============================================================================
JointGroupCommandController::~JointGroupCommandController() {}

//=============================================================================
bool JointGroupCommandController::init(hardware_interface::RobotHW *robot,
                                     ros::NodeHandle &n) {
  return initController(robot, n);
}

//=============================================================================
void JointGroupCommandController::starting(const ros::Time &time) {
  startController(time);
}

//=============================================================================
void JointGroupCommandController::stopping(const ros::Time &time) {
  stopController(time);
}

//=============================================================================
void JointGroupCommandController::update(const ros::Time &time,
                                       const ros::Duration &period) {
  updateStep(time, period);
}

//=============================================================================
bool JointGroupCommandController::shouldAcceptRequests() { return isRunning(); }

} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(rewd_controllers::JointGroupCommandController,
                       controller_interface::ControllerBase)
