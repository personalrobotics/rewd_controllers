#include <rewd_controllers/MoveUntilTouchController.hpp>

#include <functional>
#include <pluginlib/class_list_macros.h>

namespace rewd_controllers {
//=============================================================================
MoveUntilTouchController::MoveUntilTouchController()
    : MultiInterfaceController{true} // allow_optional_interfaces
      ,
      JointTrajectoryControllerBase{}, mTaringCompleted{false},
      mForceThreshold{0.0}, mTorqueThreshold{0.0} {}

//=============================================================================
MoveUntilTouchController::~MoveUntilTouchController() {}

//=============================================================================
bool MoveUntilTouchController::init(hardware_interface::RobotHW *robot,
                                    ros::NodeHandle &nh) {
  // check that doubles are lock-free atomics
  if (!mForceThreshold.is_lock_free()) {
    ROS_ERROR("Double atomics not lock-free on this system. Cannot guarantee "
              "realtime safety.");
    return false;
  }

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
  if (!nh.getParam("sensor_force_limit", mForceLimit)) {
    ROS_ERROR("Failed to load 'sensor_force_limit' paramter.");
    return false;
  }
  if (!nh.getParam("sensor_torque_limit", mTorqueLimit)) {
    ROS_ERROR("Failed to load 'sensor_torque_limit' paramter.");
    return false;
  }
  if (mForceLimit < 0) {
    ROS_ERROR("sensor_force_limit must be positive or zero");
    return false;
  }
  if (mTorqueLimit < 0) {
    ROS_ERROR("sensor_torque_limit must be positive or zero");
    return false;
  }

  // get hardware handles
  const auto ft_interface =
      robot->get<hardware_interface::ForceTorqueSensorInterface>();
  if (!ft_interface) {
    ROS_ERROR_STREAM("RobotHW has no 'ForceTorqueSensorInterface'.");
    return false;
  }
  try {
    mForceTorqueHandle = ft_interface->getHandle(ft_wrench_name);
    ROS_INFO_STREAM("Reading force/torque data from '" << ft_wrench_name
                                                       << "'.");
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Unable to get 'ForceTorqueSensorHandle' for '"
                     << ft_wrench_name << "'.");
    return false;
  }

  const auto tare_interface =
      robot->get<pr_hardware_interfaces::TriggerableInterface>();
  if (!tare_interface) {
    ROS_ERROR_STREAM("RobotHW has no 'TriggerableInterface'.");
    return false;
  }
  try {
    mTareHandle = tare_interface->getHandle(ft_tare_name);
    ROS_INFO_STREAM("Triggering tares on '" << ft_tare_name << "'.");
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Unable to get 'TriggerHandle' for '" << ft_tare_name
                                                           << "'.");
    return false;
  }

  // start action server
  mFTThresholdActionServer.reset(
      new actionlib::ActionServer<SetFTThresholdAction>{
          nh, "set_forcetorque_threshold",
          std::bind(&MoveUntilTouchController::setForceTorqueThreshold, this,
                    std::placeholders::_1),
          false});
  mFTThresholdActionServer->start();

  // initialize base trajectory controller
  return initController(robot, nh);
}

//=============================================================================
void MoveUntilTouchController::starting(const ros::Time &time) {
  // start asynchronous tare request
  mTareHandle.trigger();
  // start base trajectory controller
  startController(time);
}

//=============================================================================
void MoveUntilTouchController::stopping(const ros::Time &time) {
  // stop base trajectory controller
  stopController(time);
}

//=============================================================================
void MoveUntilTouchController::update(const ros::Time &time,
                                      const ros::Duration &period) {
  // check async tare request
  mTaringCompleted.store(mTareHandle.isTriggerComplete());
  // update base trajectory controller
  updateStep(time, period);
}

//=============================================================================
bool MoveUntilTouchController::shouldAcceptRequests() { return isRunning(); }

//=============================================================================
bool MoveUntilTouchController::shouldStopExecution(std::string &message) {
  // inelegent to just terminate any running trajectory,
  // but we must guarantee taring completes before starting
  if (!mTaringCompleted.load()) {
    return true;
  }

  const double *fArray = mForceTorqueHandle.getForce();
  if (fArray == nullptr) {
    return true;
  }
  Eigen::Map<const Eigen::Vector3d> force{fArray};

  const double *tArray = mForceTorqueHandle.getTorque();
  if (tArray == nullptr) {
    return true;
  }
  Eigen::Map<const Eigen::Vector3d> torque{tArray};

  double forceThreshold = mForceThreshold.load();
  double torqueThreshold = mTorqueThreshold.load();

  bool forceThresholdExceeded = force.norm() >= forceThreshold;
  bool torqueThresholdExceeded = torque.norm() >= torqueThreshold;

  return forceThresholdExceeded || torqueThresholdExceeded;
}

//=============================================================================
void MoveUntilTouchController::setForceTorqueThreshold(
    FTThresholdGoalHandle gh) {
  const auto goal = gh.getGoal();
  ROS_INFO_STREAM("Setting thresholds: force = " << goal->force_threshold
                                                 << ", torque = "
                                                 << goal->torque_threshold);
  FTThresholdResult result;
  result.success = true;
  if (!isRunning()) {
    result.success = false;
    result.message = "Controller not started.";
    gh.setAccepted();
    gh.setAborted(result);
    return;
  }
  // check initial taring is complete
  // NOTE: this is a hacky way to hopefully force the user to wait until taring
  // completes before using the controller, without overriding the ActionServer
  // goal accept/reject logic in JointTrajectoryControllerBase.
  else if (!mTaringCompleted.load()) {
    result.success = false;
    result.message =
        "Must wait until initial taring of force/torque sensor is complete "
        "before setting thresholds or sending trajectories.";
    gh.setAccepted();
    gh.setAborted(result);
    return;
  }
  // check threshold validity
  else if (goal->force_threshold > mForceLimit) {
    result.success = false;
    result.message = "Force threshold exceeds maximum sensor value.";
  } else if (goal->force_threshold < 0.0) {
    result.success = false;
    result.message = "Force threshold must be positive or zero.";
  } else if (goal->torque_threshold > mTorqueLimit) {
    result.success = false;
    result.message = "Torque threshold exceeds maximum sensor value.";
  } else if (goal->torque_threshold < 0.0) {
    result.success = false;
    result.message = "Torque threshold must be positive or zero.";
  }

  if (!result.success) {
    gh.setRejected(result);
  } else {
    gh.setAccepted();
    mForceThreshold.store(goal->force_threshold);
    mTorqueThreshold.store(goal->torque_threshold);
    gh.setSucceeded(result);
  }
}
} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(rewd_controllers::MoveUntilTouchController,
                       controller_interface::ControllerBase)
