#include "rewd_controllers/FTThresholdClient.hpp"
#include <thread>

namespace rewd_controllers {

using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
using FTThresholdActionClient
    = actionlib::SimpleActionClient<SetFTThresholdAction>;

//=============================================================================
FTThresholdClient::FTThresholdClient(const std::string& controllerThresholdTopic)
{
  mFTThresholdActionClient = std::unique_ptr<FTThresholdActionClient>(
      new FTThresholdActionClient(controllerThresholdTopic));
  ROS_INFO("Waiting for FT Threshold Action Server to start...");
  mFTThresholdActionClient->waitForServer();
  ROS_INFO("FT Threshold Action Server started.");
}

//=============================================================================
bool FTThresholdClient::trySetThresholdsRepeatedly(double forceThreshold, double torqueThreshold)
{
  while (ros::ok())
  {
    try {
    if (setThresholds(forceThreshold, torqueThreshold))
      return true;
    } catch (std::runtime_error) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return false;
}

//=============================================================================
bool FTThresholdClient::setThresholds(double forceThreshold, double torqueThreshold, double timeout)
{
  pr_control_msgs::SetForceTorqueThresholdGoal goal;
  goal.force_threshold = forceThreshold;
  goal.torque_threshold = torqueThreshold;
  mFTThresholdActionClient->sendGoal(goal);
  bool finished_before_timeout
      = mFTThresholdActionClient->waitForResult(ros::Duration(timeout));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state
        = mFTThresholdActionClient->getState();
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
      throw std::runtime_error(
          "F/T Thresholds could not be set: " + state.toString() + ", "
          + mFTThresholdActionClient->getResult()->message);
    }
    return true;
  }
  else
  {
    return false;
  }
}

}
