#ifndef REWD_CONTROLLERS_JOINTADAPTER_HPP_
#define REWD_CONTROLLERS_JOINTADAPTER_HPP_

#include <ros/time.h>
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_box.h>

namespace rewd_controllers {

class JointAdapter
{
public:
  JointAdapter();

  virtual ~JointAdapter();

  virtual bool initialize(const ros::NodeHandle& nodeHandle) = 0;

  virtual void update(
    const ros::Time& time, const ros::Duration& period,
    double actualPosition, double desiredPosition,
    double actualVelocity, double desiredVelocity,
    double nominalEffort) = 0;

  virtual void reset() = 0;
};

//=============================================================================
class JointPositionAdapter : public JointAdapter
{
public:
  JointPositionAdapter(hardware_interface::JointHandle positionHandle,
    dart::dynamics::DegreeOfFreedom* dof);

  bool initialize(const ros::NodeHandle& nodeHandle) override;

  void update(
    const ros::Time& time, const ros::Duration& period,
    double actualPosition, double desiredPosition,
    double actualVelocity, double desiredVelocity,
    double nominalEffort) override;

  void reset() override;

private:
  hardware_interface::JointHandle mPositionHandle;
  dart::dynamics::DegreeOfFreedom* mDof;
};

//=============================================================================
class JointVelocityAdapter : public JointAdapter
{
public:
  JointVelocityAdapter(hardware_interface::JointHandle velocityHandle,
    dart::dynamics::DegreeOfFreedom* dof);

  bool initialize(const ros::NodeHandle& nodeHandle) override;

  void update(
    const ros::Time& time, const ros::Duration& period,
    double actualPosition, double desiredPosition,
    double actualVelocity, double desiredVelocity,
    double nominalEffort) override;

  void reset() override;

private:
  hardware_interface::JointHandle mVelocityHandle;
  dart::dynamics::DegreeOfFreedom* mDof;
  control_toolbox::Pid mPid;
};

//=============================================================================
class JointEffortAdapter : public JointAdapter
{
public:
  JointEffortAdapter(hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom* dof);

  bool initialize(const ros::NodeHandle& nodeHandle) override;

  void update(
    const ros::Time& time, const ros::Duration& period,
    double actualPosition, double desiredPosition,
    double actualVelocity, double desiredVelocity,
    double nominalEffort) override;

  void reset() override;

private:
  hardware_interface::JointHandle mEffortHandle;
  dart::dynamics::DegreeOfFreedom* mDof;
  control_toolbox::Pid mPid;
};

} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_JOINTADAPTER_HPP_
