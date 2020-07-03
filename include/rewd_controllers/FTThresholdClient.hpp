#ifndef REWD_CONTROLLERS_FTTHRESHOLDCLIENT_HPP
#define REWD_CONTROLLERS_FTTHRESHOLDCLIENT_HPP

#include <actionlib/client/simple_action_client.h>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <ros/ros.h>

namespace rewd_controllers {

/// The FTThresholdClient configures the MoveUntilTouch(Topic)Controller's
/// thresholds.
/// When those thresholds are exceeded, the controller stops the movement.
class FTThresholdClient {

public:
  /// Constructor.
  FTThresholdClient(const std::string &controllerThresholdTopic);

  /// Sets the MoveUntilTouchControllers Thresholds.
  /// Blocks until the threshold could be set successfully.
  /// Can be aborted with !ros::ok(), in which case it returns false
  bool trySetThresholdsRepeatedly(double forceThreshold,
                                  double torqueThreshold);

  /// Sets the MoveUntilTouchControllers thresholds.
  /// Returns true if the thresholds were set successfully.
  bool setThresholds(double forceThreshold, double torqueThreshold,
                     double timeout = 3.0);

private:
  std::unique_ptr<actionlib::SimpleActionClient<
      pr_control_msgs::SetForceTorqueThresholdAction>>
      mFTThresholdActionClient;
};
} // namespace rewd_controllers

#endif
