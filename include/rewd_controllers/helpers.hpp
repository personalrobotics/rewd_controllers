#ifndef REWD_CONTROLLERS_HELPERS_HPP_
#define REWD_CONTROLLERS_HELPERS_HPP_
#include "JointAdapter.hpp"
#include "JointAdapterFactory.hpp"
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <string>
#include <typeinfo>
#include <vector>

namespace rewd_controllers {

//=============================================================================
struct JointParameter {
  std::string mName;
  std::string mType;
};

//=============================================================================
class SkeletonJointStateUpdater final {
public:
  SkeletonJointStateUpdater(
      dart::dynamics::SkeletonPtr skeleton,
      hardware_interface::JointStateInterface *jointStateInterface);

  void update();

private:
  dart::dynamics::SkeletonPtr mSkeleton;
  Eigen::VectorXd mDefaultPosition;
  Eigen::VectorXd mDefaultVelocity;
  Eigen::VectorXd mDefaultEffort;
  std::vector<hardware_interface::JointStateHandle> mHandles;
};

//=============================================================================
dart::dynamics::SkeletonPtr
loadRobotFromParameter(const ros::NodeHandle &nodeHandle,
                       const std::string &nameParameter);

std::vector<JointParameter>
loadJointsFromParameter(const ros::NodeHandle &nodeHandle,
                        const std::string &jointsParameter,
                        const std::string &defaultType);

/// These constraints specify the maximum final error tolerance for each joint
/// in order for the trajectory goal to be considered successful. If this
/// constraint is violated, the goal is aborted.
///
/// Specified by the ROS parameter constraints/<joint>/goal.
std::unordered_map<std::string, double> loadGoalConstraintsFromParameter(
    const ros::NodeHandle &nodeHandle,
    const std::vector<JointParameter> &jointParameters);

/// These constraints specify the error tolerance for each joint at any point
/// during execution for the trajectory goal to be considered successful. If
/// this constraint is violated, the goal is aborted.
///
/// Specified by the ROS parameter constraints/<joint>/trajectory.
std::unordered_map<std::string, double> loadTrajectoryConstraintsFromParameter(
  const ros::NodeHandle& nodeHandle,
  const std::vector<JointParameter>& jointParameters);

dart::dynamics::MetaSkeletonPtr
getControlledMetaSkeleton(const dart::dynamics::SkeletonPtr &skeleton,
                          const std::vector<JointParameter> &parameters,
                          const std::string &name);

ros::NodeHandle
createDefaultAdapterNodeHandle(const ros::NodeHandle &parentNodeHandle,
                               const dart::dynamics::DegreeOfFreedom *dof);

} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_HELPERS_HPP_
