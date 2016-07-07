#ifndef REWD_CONTROLLERS_JOINTADAPTER_HPP_
#define REWD_CONTROLLERS_JOINTADAPTER_HPP_
#include <ros/time.h>
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

namespace rewd_controllers {

class JointAdapter
{
public:
  JointAdapter();

  virtual ~JointAdapter();

  virtual void setDesiredPosition(double desiredPosition);

  virtual bool initialize(const ros::NodeHandle& nodeHandle) = 0;

  virtual void update(const ros::Time& time, const ros::Duration& period,
    double desiredPosition) = 0;

  virtual void reset() = 0;

protected:
  double mDesiredPosition;
};

//=============================================================================
class JointPositionAdapter : public JointAdapter
{
public:
  JointPositionAdapter(hardware_interface::JointHandle positionHandle,
    dart::dynamics::DegreeOfFreedom* dof);

  bool initialize(const ros::NodeHandle& nodeHandle) override;

  void update(const ros::Time& time, const ros::Duration& period,
    double desiredPosition) override;

  void reset() override;

private:
  hardware_interface::JointHandle mPositionHandle;
  dart::dynamics::DegreeOfFreedom* mDof;
};

//=============================================================================
class JointEffortAdapter : public JointAdapter
{
public:
  JointEffortAdapter(hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom* dof);

  bool initialize(const ros::NodeHandle& nodeHandle) override;

  void update(const ros::Time& time, const ros::Duration& period,
    double desiredPosition) override;

  void reset() override;

private:
  hardware_interface::JointHandle mEffortHandle;
  dart::dynamics::DegreeOfFreedom* mDof;
  control_toolbox::Pid mPid;
};

} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_JOINTADAPTER_HPP_
