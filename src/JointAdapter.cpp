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
  const ros::Time& time, const ros::Duration& period,
  double desiredPosition, double desiredVelocity)
{
  // Simple pass-through interface.
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
  const ros::Time& time, const ros::Duration& period,
  double desiredPosition, double desiredVelocity)
{
  // TODO: Use Aikido to compute this error.
  const auto actualPosition = mDof->getPosition();
  double errorPosition;
  if (mDof->isCyclic())
  {
    errorPosition = angles::shortest_angular_distance(
      desiredPosition, actualPosition);
  }
  else
  {
    errorPosition = desiredPosition - actualPosition;
  }

  const auto errorVelocity = mDof->getVelocity() - desiredVelocity;
  const auto velocity = mPid.computeCommand(
    errorPosition, errorVelocity, period);

  mVelocityHandle.setCommand(velocity);
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
  const ros::Time& time, const ros::Duration& period,
  double desiredPosition, double desiredVelocity)
{
  // This is the inverse dynamics torque computed by DART.
  const auto effortInverseDynamics = mDof->getForce();

  // Compute an error torque using PID.
  // TODO: Use Aikido to compute this error.
  const auto actualPosition = mDof->getPosition();
  double errorPosition;
  if (mDof->isCyclic())
  {
    errorPosition = angles::shortest_angular_distance(
      desiredPosition, actualPosition);
  }
  else
  {
    errorPosition = desiredPosition - actualPosition;
  }

  // TODO: Handle wrapping in this error calculation.
  const auto errorVelocity = desiredVelocity - mDof->getVelocity();

  const auto effortPid = mPid.computeCommand(
    errorPosition, errorVelocity, period);
  const auto effortTotal = effortInverseDynamics + effortPid;

  mEffortHandle.setCommand(effortTotal);
}

//=============================================================================
void JointEffortAdapter::reset()
{
  mPid.reset();
}

} // namespace rewd_controllers
