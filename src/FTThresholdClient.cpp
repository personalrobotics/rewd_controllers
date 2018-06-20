#include "rewd_controllers/FTThresholdClient.hpp"
#include <thread>

namespace rewd_controllers {

using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
using FTThresholdActionClient
    = actionlib::SimpleActionClient<SetFTThresholdAction>;

FTThresholdClient::FTThresholdClient(const std::string& controllerThresholdTopic, ros::NodeHandle nodeHandle)
  : nodeHandle(nodeHandle)
{
  ftThresholdActionClient = std::unique_ptr<FTThresholdActionClient>(
      new FTThresholdActionClient(controllerThresholdTopic));
  ROS_INFO("Waiting for FT Threshold Action Server to start...");
  ftThresholdActionClient->waitForServer();
  ROS_INFO("FT Threshold Action Server started.");
}

void FTThresholdClient::trySetThresholdRepeatedly(double forceThreshold, double torqueThreshold)
{
  bool setFTSuccessful = false;
  while (!setFTSuccessful)
  {
    setFTSuccessful = trySetThreshold(forceThreshold, torqueThreshold);
    if (setFTSuccessful)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (!ros::ok())
    {
      exit(0);
    }
  }
}

bool FTThresholdClient::trySetThreshold(double forceThreshold, double torqueThreshold, double timeout)
{
  try
  {
    setThreshold(forceThreshold, torqueThreshold, timeout);
  }
  catch (std::runtime_error)
  {
    return false;
  }
  return true;
}

void FTThresholdClient::setThreshold(double forceThreshold, double torqueThreshold, double timeout)
{
  pr_control_msgs::SetForceTorqueThresholdGoal goal;
  goal.force_threshold = forceThreshold;
  goal.torque_threshold = torqueThreshold;
  ftThresholdActionClient->sendGoal(goal);
  bool finished_before_timeout
      = ftThresholdActionClient->waitForResult(ros::Duration(timeout));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state
        = ftThresholdActionClient->getState();
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
      throw std::runtime_error(
          "F/T Thresholds could not be set: " + state.toString() + ", "
          + ftThresholdActionClient->getResult()->message);
    }
  }
  else
  {
    throw std::runtime_error("F/T Thresholds could not be set: Timeout");
  }
}
}
