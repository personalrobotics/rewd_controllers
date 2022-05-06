#include <rewd_controllers/GravityCompensationController.hpp>

#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

namespace rewd_controllers {
//=============================================================================
GravityCompensationController::GravityCompensationController()
    : MultiInterfaceController(true) // allow_optional_interfaces
{}

//=============================================================================
GravityCompensationController::~GravityCompensationController() {}

//=============================================================================
bool GravityCompensationController::init(hardware_interface::RobotHW *robot,
                                         ros::NodeHandle &n) {
  using hardware_interface::EffortJointInterface;
  using hardware_interface::JointStateInterface;

  // Build up the list of controlled DOFs.
  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto &param : jointParameters) {
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

    if (param.mType != "effort") {
      ROS_ERROR_STREAM("Joint '"
                       << param.mName
                       << "' is not effort-controlled and cannot be "
                          "used in a gravity compensation controller");
      return false;
    }
  }

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton)
    return false;

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton =
      getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton)
    return false;

  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface) {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
      new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  const auto numControlledDofs = mControlledSkeleton->getNumDofs();

  mControlledJointHandles.resize(numControlledDofs);
  mCalculatedForces.resize(numControlledDofs);

  const auto effortJointInterface = robot->get<EffortJointInterface>();
  if (!effortJointInterface) {
    ROS_ERROR("Unable to get EffortJointInterface from RobotHW instance.");
    return false;
  }

  for (size_t idof = 0; idof < numControlledDofs; ++idof) {
    const auto dofName = mControlledSkeleton->getDof(idof)->getName();
    try {
      auto handle = effortJointInterface->getHandle(dofName);
      mControlledJointHandles[idof] = handle;
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM(
          "Unable to get interface of type 'EffortJointInterface' for joint '"
          << dofName << "'.");
      return false;
    }
  }

  ROS_INFO("GravityCompensationController initialized successfully");
  return true;
}

//=============================================================================
void GravityCompensationController::update(const ros::Time &time,
                                           const ros::Duration &period) {

  Eigen::VectorXd old_efforts;
  old_efforts = mControlledSkeleton->getForces();
  std::cout<<"old_efforts: ";
  for(int i=0; i<old_efforts.size(); i++)
    std::cout<<old_efforts(i)<<" ";
  std::cout<<std::endl;

  mSkeletonUpdater->update();

  Eigen::VectorXd actual_position, actual_velocity, actual_accelaration, actual_efforts, gravity_forces, desired_gravity_forces;
  actual_position = mControlledSkeleton->getPositions();
  actual_velocity = mControlledSkeleton->getVelocities();
  actual_accelaration = mControlledSkeleton->getAccelerations();
  actual_efforts = mControlledSkeleton->getForces();
  gravity_forces = mControlledSkeleton->getCoriolisAndGravityForces();

  Eigen::Vector3d gravity = mSkeleton->getGravity();

  std::cout<<"gravity: ";
  for(int i=0; i<gravity.size(); i++)
    std::cout<<gravity(i)<<" ";
  std::cout<<std::endl;

  std::cout<<"actual_position: ";
  for(int i=0; i<actual_position.size(); i++)
    std::cout<<actual_position(i)<<" ";
  std::cout<<std::endl;
  std::cout<<"actual_velocity: ";
  for(int i=0; i<actual_velocity.size(); i++)
    std::cout<<actual_velocity(i)<<" ";
  std::cout<<std::endl;
  std::cout<<"actual_accelaration: ";
  for(int i=0; i<actual_accelaration.size(); i++)
    std::cout<<actual_accelaration(i)<<" ";
  std::cout<<std::endl;
  std::cout<<"actual_efforts: ";
  for(int i=0; i<actual_efforts.size(); i++)
    std::cout<<actual_efforts(i)<<" ";
  std::cout<<std::endl;
  std::cout<<"gravity_forces: ";
  for(int i=0; i<gravity_forces.size(); i++)
    std::cout<<gravity_forces(i)<<" ";
  std::cout<<std::endl;

  Eigen::VectorXd external_forces;
  external_forces = mControlledSkeleton->getExternalForces();
  std::cout<<"external_forces: ";
  for(int i=0; i<external_forces.size(); i++)
    std::cout<<external_forces(i)<<" ";
  std::cout<<std::endl;

  // Compute inverse dynamics torques for the current configuration
  mSkeleton->computeInverseDynamics();

  auto sixthBodyNode = mSkeleton->getBodyNode(6);
  auto fifthBodyNode = mSkeleton->getBodyNode(5);

  std::cout<<"Number of body nodes: "<<mSkeleton->getNumBodyNodes()<<std::endl;

  for(int i=0; i<mSkeleton->getNumBodyNodes(); i++)
    std::cout<<"Node name: "<<mSkeleton->getBodyNode(i)->getName()<<" Mass:"<<mSkeleton->getBodyNode(i)->getInertia().getMass()<<
      " Gravity Mode:"<<mSkeleton->getBodyNode(i)->getGravityMode()<<std::endl;

  mCalculatedForces = mControlledSkeleton->getForces();
  desired_gravity_forces = mControlledSkeleton->getCoriolisAndGravityForces();

  std::cout<<"mCalculatedForces: ";
  for(int i=0; i<mCalculatedForces.size(); i++)
    std::cout<<mCalculatedForces(i)<<" ";
  std::cout<<std::endl;

  std::cout<<"desired_gravity_forces: ";
  for(int i=0; i<desired_gravity_forces.size(); i++)
    std::cout<<desired_gravity_forces(i)<<" ";
  std::cout<<std::endl;

  Eigen::VectorXd desired_external_forces;
  desired_external_forces = mControlledSkeleton->getExternalForces();
  std::cout<<"desired_external_forces: ";
  for(int i=0; i<desired_external_forces.size(); i++)
    std::cout<<desired_external_forces(i)<<" ";
  std::cout<<std::endl;

  std::cout<<"Efforts sent: ";
  for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof) {
    std::cout<<mCalculatedForces[idof]<<" ";
    auto jointHandle = mControlledJointHandles[idof];
    jointHandle.setCommand(mCalculatedForces[idof]);
  }
  std::cout<<std::endl;
}

} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(rewd_controllers::GravityCompensationController,
                       controller_interface::ControllerBase)
