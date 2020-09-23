#include "rewd_controllers/FTThresholdClient.hpp"
#include <thread>

namespace rewd_controllers {

using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
using FTThresholdActionClient =
    actionlib::SimpleActionClient<SetFTThresholdAction>;

//=============================================================================
FTThresholdClient::FTThresholdClient(
    const std::string &controllerThresholdTopic) {
  mFTThresholdActionClient = std::unique_ptr<FTThresholdActionClient>(
      new FTThresholdActionClient(controllerThresholdTopic));
  ROS_INFO("Waiting for FT Threshold Action Server to start...");
  mFTThresholdActionClient->waitForServer();
  ROS_INFO("FT Threshold Action Server started.");
}

//=============================================================================
bool FTThresholdClient::setThresholds(double forceThreshold,
                                      double torqueThreshold, bool retare,
                                      double timeout) {
  pr_control_msgs::SetForceTorqueThresholdGoal goal;
  goal.force_threshold = forceThreshold;
  goal.torque_threshold = torqueThreshold;
  goal.retare = retare;
  mFTThresholdActionClient->sendGoal(goal);

  // Ignore timeout if re-taring
  auto duration = retare ? ros::Duration(0.0) : ros::Duration(timeout);
  bool finished_before_timeout =
      mFTThresholdActionClient->waitForResult(duration);

  actionlib::SimpleClientGoalState state = mFTThresholdActionClient->getState();

  if (!finished_before_timeout) {
    return false;
  } else if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
    return true;
  } else if (state == actionlib::SimpleClientGoalState::StateEnum::ABORTED) {
    return false;
  } else {
    throw std::runtime_error(
        "F/T Thresholds could not be set: " + state.toString() + ", " +
        mFTThresholdActionClient->getResult()->message);
  }
}

} // namespace rewd_controllers
