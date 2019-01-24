#include <rewd_controllers/GravityCompensationController.hpp>

#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

namespace rewd_controllers {
//=============================================================================
GravityCompensationController::GravityCompensationController()
  : MultiInterfaceController(true) // allow_optional_interfaces
{
}

//=============================================================================
GravityCompensationController::~GravityCompensationController()
{
}

//=============================================================================
bool GravityCompensationController::init(
    hardware_interface::RobotHW* robot, ros::NodeHandle& n)
{
  using hardware_interface::JointStateInterface;
  using hardware_interface::EffortJointInterface;

  // Build up the list of controlled DOFs.
  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto& param : jointParameters)
  {
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

    if (param.mType != "effort")
    {
      ROS_ERROR_STREAM(
          "Joint '" << param.mName
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
  mControlledSkeleton
      = getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton)
    return false;

  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface)
  {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
      new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  const auto numControlledDofs = mControlledSkeleton->getNumDofs();

  mControlledJointHandles.resize(numControlledDofs);
  mCalculatedForces.resize(numControlledDofs);

  const auto effortJointInterface = robot->get<EffortJointInterface>();
  if (!effortJointInterface)
  {
    ROS_ERROR("Unable to get EffortJointInterface from RobotHW instance.");
    return false;
  }

  for (size_t idof = 0; idof < numControlledDofs; ++idof)
  {
    const auto dofName = mControlledSkeleton->getDof(idof)->getName();
    try
    {
      auto handle = effortJointInterface->getHandle(dofName);
      mControlledJointHandles[idof] = handle;
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM(
          "Unable to get interface of type 'EffortJointInterface' for joint '"
          << dofName
          << "'.");
      return false;
    }
  }

  ROS_INFO("GravityCompensationController initialized successfully");
  return true;
}

//=============================================================================
void GravityCompensationController::update(
    const ros::Time& time, const ros::Duration& period)
{
  mSkeletonUpdater->update();

  // Compute inverse dynamics torques for the current configuration
  mSkeleton->computeInverseDynamics();
  mCalculatedForces = mControlledSkeleton->getForces();

  for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof)
  {
    auto jointHandle = mControlledJointHandles[idof];
    jointHandle.setCommand(mCalculatedForces[idof]);
  }
}

} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(
    rewd_controllers::GravityCompensationController,
    controller_interface::ControllerBase)
