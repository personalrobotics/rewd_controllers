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
                                  double /*nominalEffort*/,
                                  int /*dof*/) {
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
                                  double /*nominalEffort*/,
                                  int /*dof*/) {
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
                                double nominalEffort,
                                int /*dof*/) {
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
                                  double nominalEffort,
                                  int /*dof*/) {
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
  mExtendedJoints = new ExtendedJointPosition(1, 3 * M_PI / 2);

  mJointStiffnessMatrix.resize(7, 7);
  mJointStiffnessMatrix.setZero();
  Eigen::VectorXd joint_stiffness_vec(7);
  // joint_stiffness_vec << 3000,3000,3000,3000,2000,2000,2000;
  joint_stiffness_vec << 7000, 7000, 7000, 7000, 5000, 5000, 7000;
  // joint_stiffness_vec << 100,100,100,100,80,80,80;
  mJointStiffnessMatrix.diagonal() = joint_stiffness_vec;

  mRotorInertiaMatrix.resize(7, 7);
  mRotorInertiaMatrix.setZero();
  Eigen::VectorXd rotor_inertia_vec(7);
  // joint_stiffness_vec << 3000,3000,3000,3000,2000,2000,2000;
  rotor_inertia_vec << 0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
  mRotorInertiaMatrix.diagonal() = rotor_inertia_vec;
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
                                double actualEffort, double nominalEffort, int dof) {
  // TODO: Handle position wrapping on SO(2) joints.
  // TODO: Convert all calculations to per-joint
  // k, d, l, lp, stiffness, rotor_inertia
  // if (dof != 6 && dof != 5 && dof != 4 && dof != 0) {
  //   mEffortHandle.setCommand(nominalEffort);
  //   return;
  // }
  if (!is_initialized){
    mLastDesiredPosition = desiredPosition;
    is_initialized = true;
    mExtendedJoints->initializeExtendedJointPosition(desiredPosition, dof);
    mExtendedJoints->estimateExtendedJoint(desiredPosition, dof);
    nominal_theta_prev_ = mExtendedJoints->getExtendedJoint(dof);
    nominal_theta_dot_prev_ = actualVelocity;
    mDesiredPosition = mExtendedJoints->getExtendedJoint(dof);
    // mDesiredVelocity = 0.;
    mDesiredVelocity = desiredVelocity;
  }

  // std::cout << "Joint: " << dof << " Desired Position: " << desiredPosition << std::endl;
  // if (std::abs(desiredPosition - mLastDesiredPosition) > 0.000001 && actualPosition != desiredPosition) {
  if (desiredPosition != mLastDesiredPosition && actualPosition != desiredPosition){
    mLastDesiredPosition = desiredPosition;
    // mExtendedJoints->initializeExtendedJointPosition(desiredPosition, dof);
    mExtendedJoints->estimateExtendedJoint(desiredPosition, dof);
    // nominal_theta_prev_ = mExtendedJoints->getExtendedJoint(dof);
    // nominal_theta_dot_prev_ = actualVelocity;
    mDesiredPosition = mExtendedJoints->getExtendedJoint(dof);
    // mDesiredVelocity = 0.;
    mDesiredVelocity = desiredVelocity;
  }
  
  if (std::isnan(nominalEffort))
    throw std::range_error("nominalEffort is NaN");

  // TODO: Check sign
  actualEffort = -actualEffort;

  // FO Parameters
  Eigen::MatrixXd L(7,7), Lp(7,7);
  L.setZero();
  Lp.setZero();
  // TODO: Make sure these values match the ones in Yui
  L.diagonal() << 160, 160, 160, 160, 100, 100, 100;
  Lp.diagonal() << 10, 10, 10, 10, 7.5, 7.5, 7.5;

  // Eigen::VectorXd q(7), dq(7), 
  double theta, dtheta;
  // std::cout << "Desired Position: " << mDesiredPosition << " actual position: " << actualPosition << std::endl;
  mExtendedJoints->estimateExtendedJoint(actualPosition, dof);
  theta = mExtendedJoints->getExtendedJoint(dof);

  double theta_d, dtheta_d;
  // TODO: Convert this to 1 / stiffness for the given joint
  theta_d = mDesiredPosition + nominalEffort / mJointStiffnessMatrix.coeff(dof, dof);
  dtheta_d = mDesiredVelocity;

  //compute joint torque
  double tau_d, tau_task;
  tau_task = mPid.computeCommand(theta_d - nominal_theta_prev_,
                           dtheta_d - nominal_theta_dot_prev_, period) + nominalEffort;
  // tau_task = -joint_k_mat.coeff(dof, dof) *(nominal_theta_prev_-theta_d) - joint_d_mat.coeff(dof, dof) *(nominal_theta_dot_prev_ - dtheta_d) + nominalEffort;
  // TODO: Use mPID object for computation
  // const auto pidEffort =
  //     mPid.computeCommand(desiredPosition - actualPosition,
  //                         desiredVelocity - actualVelocity, period);
  // tau_task = nominalEffort - pidEffort;
  
  // For Friction Observer
  double step_time;
  step_time = 0.001;

  //this function update nominal plant theta
  double nominal_theta, nominal_theta_dot, nominal_theta_ddot; //nomianl plant

  //nominal motor plant dynamics
  nominal_theta_ddot = (tau_task - actualEffort) / mRotorInertiaMatrix.coeff(dof, dof);
  // std::cout << "nominal_theta_ddot: " << nominal_theta_ddot << std::endl;
  nominal_theta_dot = nominal_theta_dot_prev_ + nominal_theta_ddot * step_time;
  nominal_theta = nominal_theta_prev_ + nominal_theta_dot * step_time;
  //update nominal theta
  nominal_theta_prev_ = nominal_theta;
  nominal_theta_dot_prev_ = nominal_theta_dot;

  // PD Friction observer
  double nominal_friction;
  // Eigen::VectorXd nominal_friction_2(7);

  nominal_friction = mRotorInertiaMatrix.coeff(dof, dof)*L.coeff(dof, dof)*((nominal_theta_dot_prev_ - actualVelocity) + Lp.coeff(dof, dof)*(nominal_theta_prev_ - theta));
  tau_d = tau_task + nominal_friction;
  // std::cout << "==================================" << std::endl;
  // std::cout << "Joint: " << dof << "\nactualEffort: " << actualEffort << "\ngravity: " << nominalEffort << "\ntau_task: " << tau_task << "\nnominal_friction: " << nominal_friction << std::endl;
  // std::cout << " total: " << tau_d << std::endl;
  // std::cout << "----------------------------------" << std::endl;
  // std::cout << "\nNominal_theta_dot_prev_: " << nominal_theta_dot_prev_ << "\nactualVelocity: " << actualVelocity << "\nNominal_theta_prev_: " << nominal_theta_prev_ << "\ntheta: " << theta << std::endl;
  // std::cout << "==================================" << std::endl;
  // tau_d = nominal_friction + nominalEffort;
  // std::cout << "tau_d: " << tau_d << std::endl;
  // mEffortHandle.setCommand(nominalEffort);
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
                                double nominalEffort,
                                int /*dof*/) {
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
