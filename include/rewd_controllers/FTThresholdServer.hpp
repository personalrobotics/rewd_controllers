#ifndef REWD_CONTROLLERS_FTTHRESHOLDSERVER_HPP
#define REWD_CONTROLLERS_FTTHRESHOLDSERVER_HPP

#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <pr_control_msgs/TriggerAction.h>
#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <mutex>

namespace rewd_controllers {

// Default force/torque limit
static const double DEFAULT_MAX = 50.0;
// Default server name
static const std::string DEFAULT_SERVER = "set_forcetorque_threshold";

/// The FTThresholdClient configures all MoveUntilTouch- controllers'
/// thresholds.
/// When those thresholds are exceeded, the controller stops the movement.
class FTThresholdServer {

public:
  /// Constructor.
  FTThresholdServer(ros::NodeHandle &nh, const std::string &wrenchTopic,
                    const std::string &tareTopic,
                    double forceLimit = DEFAULT_MAX,
                    double torqueLimit = DEFAULT_MAX,
                    const std::string &serverName = DEFAULT_SERVER);

  /**
   * \brief Call from the update thread every control cycle, this method
   * reads the current force/torque sensor state and compares with
   * tolerances specified by the set_forcetorque_threshold service.
   *
   * \returns True if current wrench exceeds force/torque threshold. If a
   * threshold is set to `0.0`, it is ignored.
   */
  bool shouldStopExecution(std::string &message);

  /**
   * \brief Call to start internal Action Server
   */
  void start();

  /**
   * \brief Call to stop internal Action Server
   */
  void stop();

private:
  using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
  using FTThresholdActionServer = actionlib::ActionServer<SetFTThresholdAction>;
  using FTThresholdGoalHandle = FTThresholdActionServer::GoalHandle;
  using FTThresholdResult = pr_control_msgs::SetForceTorqueThresholdResult;
  using TareActionClient =
      actionlib::SimpleActionClient<pr_control_msgs::TriggerAction>;

  // \brief Node handle for re-init
  std::unique_ptr<ros::NodeHandle> mNodeHandle;

  // \brief Server name for re-init
  std::string mServerName;

  // \brief Protects mForce and mTorque from simultaneous access.
  std::mutex mForceTorqueDataMutex;

  // \brief The latest force of the sensor.
  Eigen::Vector3d mForce;

  // \brief The latest torque of the sensor.
  Eigen::Vector3d mTorque;

  // \brief Is true if the taring (or 'calibration') procedure is finished.
  std::atomic_bool mTaringCompleted;

  // \brief Sensor force limit. Cannot set a threshold higher than that.
  double mForceLimit;

  // \brief Sensor torque limit. Cannot set a threshold higher than that.
  double mTorqueLimit;

  // \brief Gets data from the force/torque sensor
  ros::Subscriber mForceTorqueDataSub;

  // \brief Starts and handles the taring (calibration) process of the sensor
  std::unique_ptr<TareActionClient> mTareActionClient;

  // \brief ActionServer that enables others to set the force/torque thresholds
  std::unique_ptr<FTThresholdActionServer> mFTThresholdActionServer;

  // \brief If the force is higher than this threshold, the controller aborts.
  std::atomic<double> mForceThreshold;

  // \brief If the torque is higher than this threshold, the controller aborts.
  std::atomic<double> mTorqueThreshold;

  // \brief Keeps track of the last update time
  std::chrono::time_point<std::chrono::steady_clock>
      mTimeOfLastSensorDataReceived;

  /**
   * \brief Callback for pr_control_msgs::SetForceTorqueThresholdAction.
   */
  void setForceTorqueThreshold(FTThresholdGoalHandle gh);

  /**
   * \brief Called whenever a new Force/Torque message arrives on the ros topic
   */
  void forceTorqueDataCallback(const geometry_msgs::WrenchStamped &msg);
};
} // namespace rewd_controllers

#endif
