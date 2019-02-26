#include <rewd_controllers/JointAdapter.hpp>

#include <angles/angles.h>
#include <cmath>
#include <stdexcept>

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
  , mLowerLimit{dof->getPositionLowerLimit()}
  , mUpperLimit{dof->getPositionUpperLimit()}
{
}

//=============================================================================
bool JointPositionAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return true; // Do nothing.
}

//=============================================================================
double JointPositionAdapter::computeCommand(
  const ros::Time& /*time*/, const ros::Duration& /*period*/,
  double /*actualPosition*/, double desiredPosition,
  double /*actualVelocity*/, double /*desiredVelocity*/,
  double /*nominalEffort*/)
{
  return desiredPosition;
}

//=============================================================================
bool JointPositionAdapter::checkCommand(double command)
{
  return mLowerLimit <= command && command <= mUpperLimit;
}

//=============================================================================
void JointPositionAdapter::update(double command)
{
  mPositionHandle.setCommand(command);
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
  , mLowerLimit{dof->getVelocityLowerLimit()}
  , mUpperLimit{dof->getVelocityUpperLimit()}
{
}

//=============================================================================
bool JointVelocityAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return mPid.init(nodeHandle);
}

//=============================================================================
double JointVelocityAdapter::computeCommand(
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

  return desiredVelocity + pidVelocity;
}

//=============================================================================
bool JointVelocityAdapter::checkCommand(double command)
{
  return mLowerLimit <= command && command <= mUpperLimit;
}

//=============================================================================
void JointVelocityAdapter::update(double command)
{
  mVelocityHandle.setCommand(command);
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
  , mLowerLimit{dof->getForceLowerLimit()}
  , mUpperLimit{dof->getForceUpperLimit()}
{
}

//=============================================================================
bool JointEffortAdapter::initialize(const ros::NodeHandle& nodeHandle)
{
  return mPid.init(nodeHandle);
}

//=============================================================================
double JointEffortAdapter::computeCommand(
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

  return nominalEffort + pidEffort;
}

//=============================================================================
bool JointEffortAdapter::checkCommand(double command)
{
  return mLowerLimit <= command && command <= mUpperLimit;
}

//=============================================================================
void JointEffortAdapter::update(double command)
{
  mEffortHandle.setCommand(command);
}

//=============================================================================
void JointEffortAdapter::reset()
{
  mPid.reset();
}

} // namespace rewd_controllers
