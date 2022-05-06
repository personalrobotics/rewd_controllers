#include <rewd_controllers/JointAdapter.hpp>

#include <angles/angles.h>
#include <cmath>
#include <stdexcept>

namespace rewd_controllers {

//=============================================================================
JointAdapter::JointAdapter() {}

//=============================================================================
JointAdapter::~JointAdapter() {}

//=============================================================================
JointPositionAdapter::JointPositionAdapter(
    hardware_interface::JointHandle positionHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mPositionHandle{positionHandle}, mDof{dof} {}

//=============================================================================
bool JointPositionAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  return true; // Do nothing.
}

//=============================================================================
void JointPositionAdapter::update(const ros::Time & /*time*/,
                                  const ros::Duration & /*period*/,
                                  double /*actualPosition*/,
                                  double desiredPosition,
                                  double /*actualVelocity*/,
                                  double /*desiredVelocity*/,
                                  double /*nominalEffort*/) {
  mPositionHandle.setCommand(desiredPosition);
}

//=============================================================================
void JointPositionAdapter::reset() {
  // Do nothing.
}

//=============================================================================
JointVelocityAdapter::JointVelocityAdapter(
    hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mVelocityHandle{effortHandle}, mDof{dof} {
  mUpperVelLimit = mDof->getVelocityUpperLimit();
  mLowerVelLimit = mDof->getVelocityLowerLimit();
}

//=============================================================================
bool JointVelocityAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  return mPid.init(nodeHandle);
}

//=============================================================================
void JointVelocityAdapter::update(const ros::Time & /*time*/,
                                  const ros::Duration &period,
                                  double actualPosition, double desiredPosition,
                                  double actualVelocity, double desiredVelocity,
                                  double /*nominalEffort*/) {
  const auto pidVelocity =
      mPid.computeCommand(desiredPosition - actualPosition,
                          desiredVelocity - actualVelocity, period);

  if (std::isnan(desiredVelocity))
    throw std::range_error("desiredVelocity is NaN");
  if (std::isnan(pidVelocity))
    throw std::range_error("calculated pidVelocity is NaN");

  auto commandedVelocity = pidVelocity;

  if (commandedVelocity > mUpperVelLimit ||
      commandedVelocity < mLowerVelLimit) {
    ROS_ERROR_STREAM("Velocity limit exceeded wtih desired pose "
                     << desiredPosition << " and actual pose "
                     << actualPosition);
    std::stringstream ss;
    ss << "Overall velocity [" << desiredVelocity + pidVelocity
       << "] is beyond the velocity"
       << " limits [" << mLowerVelLimit << ", " << mUpperVelLimit << "]"
       << std::endl;
    throw std::range_error(ss.str());
  }

  mVelocityHandle.setCommand(commandedVelocity);
}

//=============================================================================
void JointVelocityAdapter::reset() { mPid.reset(); }

//=============================================================================
JointEffortAdapter::JointEffortAdapter(
    hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mEffortHandle{effortHandle}, mDof{dof} {}

//=============================================================================
bool JointEffortAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  return mPid.init(nodeHandle);
}

//=============================================================================
void JointEffortAdapter::update(const ros::Time & /*time*/,
                                const ros::Duration &period,
                                double actualPosition, double desiredPosition,
                                double actualVelocity, double desiredVelocity,
                                double nominalEffort) {
  // TODO: Handle position wrapping on SO(2) joints.
  const auto pidEffort =
      mPid.computeCommand(desiredPosition - actualPosition,
                          desiredVelocity - actualVelocity, period);
  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");
  if (std::isnan(pidEffort))
    throw std::range_error("calculated pidEffort is NaN");

  ROS_INFO_STREAM("nominalEffort:"<<nominalEffort);
  ROS_INFO_STREAM("pidEffort:"<<pidEffort);
  ROS_INFO_STREAM("Sending Effort Command:"<<nominalEffort + pidEffort);

  mEffortHandle.setCommand(nominalEffort + pidEffort);
  // mEffortHandle.setCommand(0.001);
}

//=============================================================================
void JointEffortAdapter::reset() { mPid.reset(); }

//=============================================================================
JointImpedanceAdapter::JointImpedanceAdapter(
    hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mEffortHandle{effortHandle}, mDof{dof} {}

//=============================================================================
bool JointImpedanceAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  
  ROS_INFO_STREAM("Initializing Impedance Controller!");
  if (!nodeHandle.getParam("k", mKgain)) {
    ROS_ERROR("Failed to load 'k' gain parameter.");
    return false;
  }

  if (!nodeHandle.getParam("d", mDgain)) {
    ROS_ERROR("Failed to load 'd' gain parameter.");
    return false;
  }

  return true;
}

//=============================================================================
void JointImpedanceAdapter::update(const ros::Time & /*time*/,
                                const ros::Duration &period,
                                double actualPosition, double desiredPosition,
                                double actualVelocity, double desiredVelocity,
                                double nominalEffort) {
  
  double extraEffort = mKgain*(desiredPosition-actualPosition)
                           - mDgain*actualVelocity;

  // if(extraEffort > 2)
  //   extraEffort = 2;
  // if(extraEffort < -2)
  //   extraEffort = -2;
  double impedanceEffort = extraEffort + nominalEffort;
  // double impedanceEffort = nominalEffort;

  // ROS_INFO_STREAM("Actual Position:"<<actualPosition);
  // ROS_INFO_STREAM("Desired Position:"<<desiredPosition);
  // ROS_INFO_STREAM("Sending GravComp Command:"<<nominalEffort);

  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");
  if (std::isnan(impedanceEffort))
    throw std::range_error("calculated impedanceEffort is NaN");

  // ROS_INFO_STREAM("Sending Impedance Command:"<<impedanceEffort);

  mEffortHandle.setCommand(impedanceEffort);
  // mEffortHandle.setCommand(0.0);
}

//=============================================================================
void JointImpedanceAdapter::reset() { }

} // namespace rewd_controllers
