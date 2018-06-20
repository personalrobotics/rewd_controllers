#ifndef FTTHRESHOLDCLIENT_H
#define FTTHRESHOLDCLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <ros/ros.h>

namespace rewd_controllers {

/// The FTThresholdClient configures the MoveUntilTouchController's
/// thresholds.
/// When those thresholds are exceeded, the controller stops the movement.
class FTThresholdClient
{

public:
  /// Constructor.
  FTThresholdClient(const std::string& controllerThresholdTopic, ros::NodeHandle nodeHandle);

  /// Sets the MoveUntilTouchControllers Thresholds accordingly.
  /// Blocks until the threshold could be set successfully.
  /// Can be aborted with Ctrl-C.
  void trySetThresholdRepeatedly(double forceThreshold, double torqueThreshold);

  /// Sets the MoveUntilTouchControllers Thresholds accordingly.
  /// Throws a runtime_error if we are unable to set the thresholds.
  void setThreshold(double forceThreshold, double torqueThreshold, double timeout = 3.0);

  /// Sets the MoveUntilTouchControllers thresholds accordingly.
  /// Returns whether the thresholds were set successfully.
  bool trySetThreshold(double forceThreshold, double torqueThreshold, double timeout = 3.0);

private:
  double timeout;
  ros::NodeHandle nodeHandle;
  std::unique_ptr<actionlib::
                      SimpleActionClient<pr_control_msgs::
                                             SetForceTorqueThresholdAction>>
      ftThresholdActionClient;
};
}

#endif
