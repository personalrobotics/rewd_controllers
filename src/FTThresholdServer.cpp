#include "rewd_controllers/FTThresholdServer.hpp"

namespace rewd_controllers {

//=============================================================================
FTThresholdServer::FTThresholdServer(ros::NodeHandle &nh,
                                     const std::string &wrenchTopic,
                                     const std::string &tareTopic,
                                     double forceLimit, double torqueLimit,
                                     const std::string &serverName)
    : mForceLimit{forceLimit}, mTorqueLimit{torqueLimit}, mServerName{
                                                              serverName} {
  mNodeHandle.reset(new ros::NodeHandle{nh});

  // check that doubles are lock-free atomics
  if (!mForceThreshold.is_lock_free()) {
    ROS_WARN("Double atomics not lock-free on this system. Cannot guarantee "
             "realtime safety.");
  }

  // subscribe to sensor data
  mForceTorqueDataSub = nh.subscribe(
      wrenchTopic, 1, &FTThresholdServer::forceTorqueDataCallback, this);

  // action client to kick off taring
  mTareActionClient =
      std::unique_ptr<TareActionClient>(new TareActionClient(nh, tareTopic));

  // initialize action server
  stop();

  // Set initial threshold to sensor limit
  mForceThreshold.store(forceLimit);
  mTorqueThreshold.store(torqueLimit);

  // Do not require initial taring
  mTaringCompleted.store(true);
  mTimeOfLastSensorDataReceived = std::chrono::steady_clock::now();
}

//=============================================================================
void FTThresholdServer::stop() {
  // Kill Action Server by re-instantiating it
  mFTThresholdActionServer.reset(
      new actionlib::ActionServer<SetFTThresholdAction>{
          *mNodeHandle, mServerName,
          std::bind(&FTThresholdServer::setForceTorqueThreshold, this,
                    std::placeholders::_1),
          false});
}

//=============================================================================
void FTThresholdServer::start() { mFTThresholdActionServer->start(); }

//=============================================================================
// Max F/T sensor wait time
// Arbitrary, can be changed in the future.
// ATI runs at 120Hz, Gelsight could run as slow as 10Hz
static const std::chrono::milliseconds MAX_DELAY =
    std::chrono::milliseconds(100);

bool FTThresholdServer::shouldStopExecution(std::string &message) {
  // we must guarantee taring completes before moving
  if (!mTaringCompleted.load()) {
    ROS_WARN("taring not yet completed!");
    return true;
  }

  // Watchdog
  if ((std::chrono::steady_clock::now() - mTimeOfLastSensorDataReceived >
       MAX_DELAY)) {
    message = "Lost connection to F/T sensor!";
    ROS_WARN(message.c_str());
    return true;
  }

  double forceThreshold = mForceThreshold.load();
  double torqueThreshold = mTorqueThreshold.load();

  std::lock_guard<std::mutex> lock(mForceTorqueDataMutex);
  bool forceThresholdExceeded =
      (forceThreshold != 0.0) && (mForce.norm() >= forceThreshold);
  bool torqueThresholdExceeded =
      (torqueThreshold != 0.0) && (mTorque.norm() >= torqueThreshold);

  if (forceThresholdExceeded) {
    std::stringstream messageStream;
    messageStream << "Force Threshold exceeded!   Threshold: " << forceThreshold
                  << "   Force: " << mForce.x() << ", " << mForce.y() << ", "
                  << mForce.z();
    message = messageStream.str();
    ROS_WARN(message.c_str());
  }
  if (torqueThresholdExceeded) {
    std::stringstream messageStream;
    messageStream << "Torque Threshold exceeded!   Threshold: "
                  << torqueThreshold << "   Torque: " << mTorque.x() << ", "
                  << mTorque.y() << ", " << mTorque.z();
    message = messageStream.str();
    ROS_WARN(message.c_str());
  }

  return forceThresholdExceeded || torqueThresholdExceeded;
}

//=============================================================================
void FTThresholdServer::forceTorqueDataCallback(
    const geometry_msgs::WrenchStamped &msg) {
  std::lock_guard<std::mutex> lock(mForceTorqueDataMutex);
  mForce.x() = msg.wrench.force.x;
  mForce.y() = msg.wrench.force.y;
  mForce.z() = msg.wrench.force.z;
  mTorque.x() = msg.wrench.torque.x;
  mTorque.y() = msg.wrench.torque.y;
  mTorque.z() = msg.wrench.torque.z;
  mTimeOfLastSensorDataReceived = std::chrono::steady_clock::now();
}

//=============================================================================
void FTThresholdServer::setForceTorqueThreshold(FTThresholdGoalHandle gh) {
  const auto goal = gh.getGoal();
  ROS_INFO_STREAM("Setting thresholds: force = " << goal->force_threshold
                                                 << ", torque = "
                                                 << goal->torque_threshold);
  FTThresholdResult result;
  result.success = true;

  // check threshold validity
  if (goal->force_threshold > mForceLimit) {
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
    ROS_WARN_STREAM("FTThresholdServer: " << result.message);
    gh.setRejected(result);
    return;
  }

  gh.setAccepted();

  // check that we are not already taring
  if (!mTaringCompleted.load()) {
    result.success = false;
    result.message =
        "Must wait until taring of force/torque sensor is complete "
        "before setting thresholds or sending trajectories.";
    gh.setAborted(result);
    return;
  }

  // Tare if requested
  if (goal->retare) {
    // start asynchronous tare request
    ROS_INFO("Starting Taring");
    pr_control_msgs::TriggerGoal goal;
    mTaringCompleted.store(false);
    // block until tared
    mTareActionClient->sendGoalAndWait(goal);
    ROS_INFO("Taring completed!");
    mTimeOfLastSensorDataReceived = std::chrono::steady_clock::now();
    mTaringCompleted.store(true);
  }

  // Done, store new thresholds
  mForceThreshold.store(goal->force_threshold);
  mTorqueThreshold.store(goal->torque_threshold);
  gh.setSucceeded(result);
}

} // namespace rewd_controllers