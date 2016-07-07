#ifndef REWD_CONTROLLERS_HELPERS_HPP_
#define REWD_CONTROLLERS_HELPERS_HPP_
#include <typeinfo>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "JointAdapter.hpp"
#include "JointAdapterFactory.hpp"

namespace rewd_controllers {

//=============================================================================
struct JointParameter
{
  std::string mName;
  std::string mType;
};

//=============================================================================
class SkeletonJointStateUpdater final
{
public:
  SkeletonJointStateUpdater(
    dart::dynamics::SkeletonPtr skeleton,
    hardware_interface::JointStateInterface* jointStateInterface);

  void update();

private:
  dart::dynamics::SkeletonPtr mSkeleton;
  Eigen::VectorXd mDefaultPosition;
  Eigen::VectorXd mDefaultVelocity;
  Eigen::VectorXd mDefaultEffort;
  std::vector<hardware_interface::JointStateHandle> mHandles;
};

//=============================================================================
dart::dynamics::SkeletonPtr loadRobotFromParameter(
  ros::NodeHandle& nodeHandle, const std::string& nameParameter);

std::vector<JointParameter> loadJointsFromParameter(
  ros::NodeHandle& nodeHandle, const std::string& jointsParameter,
  const std::string& defaultType);

dart::dynamics::MetaSkeletonPtr getControlledMetaSkeleton(
  const dart::dynamics::SkeletonPtr& skeleton,
  const std::vector<JointParameter>& parameters,
  const std::string& name);

} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_HELPERS_HPP_
