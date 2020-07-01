#ifndef REWD_CONTROLLERS_FTTHRESHOLDCLIENT_HPP
#define REWD_CONTROLLERS_FTTHRESHOLDCLIENT_HPP

#include <actionlib/client/simple_action_client.h>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <ros/ros.h>

namespace rewd_controllers {

/// The FTThresholdClient configures all MoveUntilTouch- controllers'
/// thresholds.
/// When those thresholds are exceeded, the controller stops the movement.
class FTThresholdClient {

public:
  /// Constructor.
  FTThresholdClient(const std::string &controllerThresholdTopic);

  /// Sets the MoveUntilTouch- thresholds.
  /// Note: timeout is ignored if re-taring.
  /// Returns true if the thresholds were set successfully.
  bool setThresholds(double forceThreshold, double torqueThreshold,
                     bool retare, double timeout = 3.0);

private:
  std::unique_ptr<actionlib::SimpleActionClient<
      pr_control_msgs::SetForceTorqueThresholdAction>>
      mFTThresholdActionClient;
};
} // namespace rewd_controllers

#endif
