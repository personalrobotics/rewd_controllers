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
                                  double /*actualEffort*/,
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
                                  double /*actualEffort*/,
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
                                double /*actualEffort*/,
                                double nominalEffort) {
  // TODO: Handle position wrapping on SO(2) joints.
  const auto pidEffort =
      mPid.computeCommand(desiredPosition - actualPosition,
                          desiredVelocity - actualVelocity, period);
  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");
  if (std::isnan(pidEffort))
    throw std::range_error("calculated pidEffort is NaN");
  double tau_task = nominalEffort + pidEffort;

  mEffortHandle.setCommand(nominalEffort + pidEffort);
}

//=============================================================================
void JointEffortAdapter::reset() { mPid.reset(); }

//=============================================================================
JointForwardEffortAdapter::JointForwardEffortAdapter(
    hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mEffortHandle{effortHandle}, mDof{dof} {}

//=============================================================================
bool JointForwardEffortAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  return true;
}

//=============================================================================
void JointForwardEffortAdapter::update(const ros::Time & /*time*/,
                                  const ros::Duration &period,
                                  double /*actualPosition*/, double /*desiredPosition*/,
                                  double /*actualVelocity*/, double /*desiredVelocity*/,
                                  double /*actualEffort*/,
                                  double nominalEffort) {
  mEffortHandle.setCommand(nominalEffort);
}

//=============================================================================
void JointForwardEffortAdapter::reset() {
  // Do nothing.
}

//=============================================================================
JointCompliantAdapter::JointCompliantAdapter(
    hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mEffortHandle{effortHandle}, mDof{dof} {
  mExtendedJoints = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);
}

//=============================================================================
bool JointCompliantAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  return mPid.init(nodeHandle);
}

//=============================================================================
void JointCompliantAdapter::update(const ros::Time & /*time*/,
                                const ros::Duration &period,
                                double actualPosition, double desiredPosition,
                                double actualVelocity, double desiredVelocity,
                                double actualEffort, double nominalEffort) {
  // TODO: Handle position wrapping on SO(2) joints.
  // TODO: Convert all calculations to per-joint
  std::cout << "pidEffort: " << pidEffort << std::endl;
  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");
  if (std::isnan(pidEffort))
    throw std::range_error("calculated pidEffort is NaN");
  // TODO: Check sign
  actualEffort = -actualEffort;

  // float tau_task = nominalEffort + pidEffort;

  Eigen::MatrixXd joint_k_mat(7,7), joint_d_mat(7,7);
  // stiffness
  joint_k_mat.setZero();
  // think of this as something that affects the speed of return to the desired position
  joint_d_mat.setZero();
  joint_k_mat.diagonal() << 50, 50, 50, 50, 50, 50, 50;
  joint_d_mat.diagonal() << 2, 2, 2, 2, 2, 2, 2;

  // FO Parameters
  Eigen::MatrixXd L(7,7), Lp(7,7);
  L.setZero();
  Lp.setZero();
  // TODO: Make sure these values match the ones in Yui
  L.diagonal() << 160, 160, 160, 160, 100, 100, 100;
  Lp.diagonal() << 10, 10, 10, 10, 7.5, 7.5, 7.5;

  Eigen::VectorXd q(7), dq(7), theta(7), dtheta(7);
  mExtendedJoints->initializeExtendedJointPosition(mActualPosition);
  mExtendedJoints->estimateExtendedJoint(mActualPosition);
  theta = mExtendedJoints->getExtendedJoint();

  Eigen::VectorXd theta_d(7), dtheta_d(7);
  // TODO: Convert this to 1 / stiffness for the given joint
  theta_d = desiredPosition + joint_stiffness_matrix_.inverse()*gravity;
  dtheta_d = desiredVelocity;

  //compute joint torque
  double tau_d, tau_task;
  tau_task = -joint_k_mat*(nominal_theta_prev_-theta_d) - joint_d_mat*(nominal_theta_dot_prev_ - dtheta_d) + nominalEffort;
  // TODO: Use mPID object for computation
  // const auto pidEffort =
  //     mPid.computeCommand(desiredPosition - actualPosition,
  //                         desiredVelocity - actualVelocity, period);
  // tau_task = nominalEffort - pidEffort;
  // For Friction Observer
  double step_time;
  step_time = 0.001;

  //this function update nominal plant theta
  Eigen::VectorXd nominal_theta(7),nominal_theta_dot(7),nominal_theta_ddot(7); //nomianl plant

  //nominal motor plant dynamics
  nominal_theta_ddot = rotor_inertia_matrix_.inverse()*(tau_task-tau_J);
  nominal_theta_dot = nominal_theta_dot_prev_ + nominal_theta_ddot*step_time;
  nominal_theta = nominal_theta_prev_ + nominal_theta_dot*step_time;

  //update nominal theta
  nominal_theta_prev_ = nominal_theta;
  nominal_theta_dot_prev_ = nominal_theta_dot;

  // PD Friction observer
  Eigen::VectorXd nominal_friction(7);
  // Eigen::VectorXd nominal_friction_2(7);

  nominal_friction = rotor_inertia_matrix_*L*((nominal_theta_dot_prev_ - actualVelocity) + Lp*(nominal_theta_prev_ - theta));

  tau_d = tau_task + nominal_friction;
  mEffortHandle.setCommand(tau_d);
}

//=============================================================================
void JointCompliantAdapter::reset() { mPid.reset(); }


//=============================================================================
JointVelocityEffortAdapter::JointVelocityEffortAdapter(
    hardware_interface::JointHandle effortHandle,
    dart::dynamics::DegreeOfFreedom *dof)
    : mEffortHandle{effortHandle}, mDof{dof} {}

//=============================================================================
bool JointVelocityEffortAdapter::initialize(const ros::NodeHandle &nodeHandle) {
  return mPid.init(nodeHandle);
}

//=============================================================================
void JointVelocityEffortAdapter::update(const ros::Time & /*time*/,
                                const ros::Duration &period,
                                double /*actualPosition*/, double /*desiredPosition*/,
                                double actualVelocity, double desiredVelocity,
                                double /*actualEffort*/,
                                double nominalEffort) {
  const auto pidEffort =
    mPid.computeCommand(desiredVelocity - actualVelocity, period);

  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");
  if (std::isnan(pidEffort))
    throw std::range_error("calculated pidEffort is NaN");

  mEffortHandle.setCommand(nominalEffort + pidEffort);
}

//=============================================================================
void JointVelocityEffortAdapter::reset() { mPid.reset(); }


} // namespace rewd_controllers
