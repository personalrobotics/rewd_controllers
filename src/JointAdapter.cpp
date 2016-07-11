#include <angles/angles.h>
#include <rewd_controllers/JointAdapter.hpp>

namespace rewd_controllers {

//=============================================================================
JointAdapter::JointAdapter()
  : mDesiredPosition{0.}
{
}

//=============================================================================
JointAdapter::~JointAdapter()
{
}

//=============================================================================
void JointAdapter::setDesiredPosition(double desiredPosition)
{
  mDesiredPosition = desiredPosition;
}

//=============================================================================
JointPositionAdapter::JointPositionAdapter(
      hardware_interface::JointHandle positionHandle,
      dart::dynamics::DegreeOfFreedom* dof)
  : mPositionHandle{positionHandle}
  , mDof{dof}
{
}

//=============================================================================
bool JointPositionAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return true; // Do nothing.
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
  mEffortHandle.setCommand(nominalEffort + pidEffort);
}

//=============================================================================
void JointEffortAdapter::reset()
{
  mPid.reset();
}

} // namespace rewd_controllers
