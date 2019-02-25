#include <rewd_controllers/JointAdapter.hpp>

#include <angles/angles.h>
#include <cmath>
#include <stdexcept>

namespace rewd_controllers {

//=============================================================================
JointAdapter::JointAdapter()
{
  // Do nothing.
}

//=============================================================================
JointAdapter::~JointAdapter()
{
  // Do nothing.
}

//=============================================================================
JointPositionAdapter::JointPositionAdapter(
      hardware_interface::JointHandle positionHandle,
      dart::dynamics::DegreeOfFreedom* dof)
  : mPositionHandle{positionHandle}
  , mDof{dof}
{
  // Do nothing.
}

//=============================================================================
bool JointPositionAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return true;
}

//=============================================================================
void JointPositionAdapter::update(
  const ros::Time& /*time*/, const ros::Duration& /*period*/,
  double /*actualPosition*/, double desiredPosition,
  double /*actualVelocity*/, double /*desiredVelocity*/,
  double /*nominalEffort*/)
{
  mPositionHandle.setCommand(desiredPosition);
}

//=============================================================================
void JointPositionAdapter::reset()
{
  // Do nothing.
}

//=============================================================================
JointVelocityAdapter::JointVelocityAdapter(
      hardware_interface::JointHandle effortHandle,
      dart::dynamics::DegreeOfFreedom* dof)
  : mVelocityHandle{effortHandle}
  , mDof{dof}
{
  // Do nothing.
}

//=============================================================================
bool JointVelocityAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return mPid.init(nodeHandle);
}

//=============================================================================
void JointVelocityAdapter::update(
    const ros::Time& /*time*/, const ros::Duration& period,
    double actualPosition, double desiredPosition,
    double actualVelocity, double desiredVelocity,
    double /*nominalEffort*/)
{
  // TODO: Handle position wrapping on SO(2) joints.
  const auto pidVelocity = mPid.computeCommand(
    desiredPosition - actualPosition,
    desiredVelocity - actualVelocity,
    period);

  if (std::isnan(desiredVelocity))
    throw std::range_error("desiredVelocity is NaN");
  if (std::isnan(pidVelocity))
    throw std::range_error("calculated pidVelocity is NaN");

  mVelocityHandle.setCommand(desiredVelocity + pidVelocity);
}

//=============================================================================
void JointVelocityAdapter::reset()
{
  mPid.reset();
}

//=============================================================================
JointEffortAdapter::JointEffortAdapter(
      hardware_interface::JointHandle effortHandle,
      dart::dynamics::DegreeOfFreedom* dof)
  : mEffortHandle{effortHandle}
  , mDof{dof}
{
  // Do nothing.
}

//=============================================================================
bool JointEffortAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return mPid.init(nodeHandle);
}

//=============================================================================
void JointEffortAdapter::update(
    const ros::Time& /*time*/, const ros::Duration& period,
    double actualPosition, double desiredPosition,
    double actualVelocity, double desiredVelocity,
    double nominalEffort)
{
  // TODO: Handle position wrapping on SO(2) joints.
  const auto pidEffort = mPid.computeCommand(
    desiredPosition - actualPosition,
    desiredVelocity - actualVelocity,
    period);
  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");
  if (std::isnan(pidEffort))
    throw std::range_error("calculated pidEffort is NaN");

  mEffortHandle.setCommand(nominalEffort + pidEffort);
}

//=============================================================================
void JointEffortAdapter::reset()
{
  mPid.reset();
}

} // namespace rewd_controllers
