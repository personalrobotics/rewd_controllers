#include <pluginlib/class_list_macros.h>
#include <rewd_controllers/JointTrajectoryController.hpp>

namespace rewd_controllers {
//=============================================================================
JointTrajectoryController::JointTrajectoryController()
  : MultiInterfaceController{true} // allow_optional_interfaces
  , JointTrajectoryControllerBase{}
{
}

//=============================================================================
JointTrajectoryController::~JointTrajectoryController()
{
}

//=============================================================================
bool JointTrajectoryController::init(
    hardware_interface::RobotHW* robot, ros::NodeHandle& n)
{
  return initController(robot, n);
}

//=============================================================================
void JointTrajectoryController::starting(const ros::Time& time)
{
  startController(time);
}

//=============================================================================
void JointTrajectoryController::stopping(const ros::Time& time)
{
  stopController(time);
}

//=============================================================================
void JointTrajectoryController::update(
    const ros::Time& time, const ros::Duration& period)
{
  updateStep(time, period);
}

//=============================================================================
bool JointTrajectoryController::shouldAcceptRequests()
{
  return isRunning();
}

} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(
    rewd_controllers::JointTrajectoryController,
    controller_interface::ControllerBase)
