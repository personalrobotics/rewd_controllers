#ifndef REWD_CONTROLLERS_JOINTADAPTER_HPP_
#define REWD_CONTROLLERS_JOINTADAPTER_HPP_
#include <control_toolbox/pid.h>
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_box.h>
#include <ros/time.h>
#include <rewd_controllers/helpers.hpp>

namespace rewd_controllers {

class JointAdapter {
public:
  JointAdapter();

  virtual ~JointAdapter();

  virtual bool initialize(const ros::NodeHandle &nodeHandle) = 0;

  virtual void update(const ros::Time &time, const ros::Duration &period,
                      double actualPosition, double desiredPosition,
                      double actualVelocity, double desiredVelocity,
                      double actualEffort, double nominalEffort, int dof) = 0;

  virtual void reset() = 0;
};

//=============================================================================
class JointPositionAdapter : public JointAdapter {
public:
  JointPositionAdapter(hardware_interface::JointHandle positionHandle,
                       dart::dynamics::DegreeOfFreedom *dof);

  bool initialize(const ros::NodeHandle &nodeHandle) override;

  void update(const ros::Time &time, const ros::Duration &period,
              double actualPosition, double desiredPosition,
              double actualVelocity, double desiredVelocity,
              double actualEffort, double nominalEffort, int dof) override;

  void reset() override;

private:
  hardware_interface::JointHandle mPositionHandle;
  dart::dynamics::DegreeOfFreedom *mDof;
};

//=============================================================================
class JointVelocityAdapter : public JointAdapter {
public:
  JointVelocityAdapter(hardware_interface::JointHandle velocityHandle,
                       dart::dynamics::DegreeOfFreedom *dof);

  bool initialize(const ros::NodeHandle &nodeHandle) override;

  void update(const ros::Time &time, const ros::Duration &period,
              double actualPosition, double desiredPosition,
              double actualVelocity, double desiredVelocity,
              double actualEffort, double nominalEffort, int dof) override;
  void reset() override;

private:
  hardware_interface::JointHandle mVelocityHandle;
  dart::dynamics::DegreeOfFreedom *mDof;
  control_toolbox::Pid mPid;

  // Storing value from DART on construction
  double mUpperVelLimit, mLowerVelLimit;
};

//=============================================================================
class JointEffortAdapter : public JointAdapter {
public:
  JointEffortAdapter(hardware_interface::JointHandle effortHandle,
                     dart::dynamics::DegreeOfFreedom *dof);

  bool initialize(const ros::NodeHandle &nodeHandle) override;

  void update(const ros::Time &time, const ros::Duration &period,
              double actualPosition, double desiredPosition,
              double actualVelocity, double desiredVelocity,
              double actualEffort, double nominalEffort, int dof) override;

  void reset() override;

private:
  hardware_interface::JointHandle mEffortHandle;
  dart::dynamics::DegreeOfFreedom *mDof;
  control_toolbox::Pid mPid;
};

//=============================================================================
class JointCompliantAdapter : public JointAdapter {
public:
  JointCompliantAdapter(hardware_interface::JointHandle effortHandle,
                     dart::dynamics::DegreeOfFreedom *dof);

  bool initialize(const ros::NodeHandle &nodeHandle) override;

  void update(const ros::Time &time, const ros::Duration &period,
              double actualPosition, double desiredPosition,
              double actualVelocity, double desiredVelocity,
              double actualEffort, double nominalEffort, int dof) override;

  void reset() override;

private:
  hardware_interface::JointHandle mEffortHandle;
  dart::dynamics::DegreeOfFreedom *mDof;
  control_toolbox::Pid mPid;
  double nominal_theta_dot_prev_;
  double nominal_theta_prev_;
  bool is_initialized = false;

  //DYNAMIC PARAMETER OF KINOVA GEN3
  Eigen::MatrixXd mJointStiffnessMatrix;
  Eigen::MatrixXd mRotorInertiaMatrix;
  double mDesiredPosition;
  double mLastDesiredPosition;
  double mDesiredVelocity;
  double mLastDesiredVelocity;

  //Initial position & orientation
  Eigen::Vector3d init_position;
  Eigen::Matrix3d init_orientation;
  Eigen::VectorXd init_q;
  Eigen::VectorXd init_dq;

  ExtendedJointPosition* mExtendedJoints;

  long long int mCount;
};

//=============================================================================
class JointForwardEffortAdapter : public JointAdapter {
public:
  JointForwardEffortAdapter(hardware_interface::JointHandle effortHandle,
                     dart::dynamics::DegreeOfFreedom *dof);

  bool initialize(const ros::NodeHandle &nodeHandle) override;

  void update(const ros::Time &time, const ros::Duration &period,
              double actualPosition, double desiredPosition,
              double actualVelocity, double desiredVelocity,
              double actualEffort, double nominalEffort, int dof) override;

  void reset() override;

private:
  hardware_interface::JointHandle mEffortHandle;
  dart::dynamics::DegreeOfFreedom *mDof;
};

//=============================================================================
class JointVelocityEffortAdapter : public JointAdapter {
public:
  JointVelocityEffortAdapter(hardware_interface::JointHandle effortHandle,
                            dart::dynamics::DegreeOfFreedom *dof);

  bool initialize(const ros::NodeHandle &nodeHandle) override;

  void update(const ros::Time &time, const ros::Duration &period,
              double actualPosition, double desiredPosition,
              double actualVelocity, double desiredVelocity,
              double actualEffort, double nominalEffort, int dof) override;

  void reset() override;

private:
  hardware_interface::JointHandle mEffortHandle;
  dart::dynamics::DegreeOfFreedom *mDof;
  control_toolbox::Pid mPid;
};


} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_JOINTADAPTER_HPP_
