#include <rewd_controllers/JointGroupPositionController.hpp>

#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pluginlib/class_list_macros.h>

namespace rewd_controllers {

//=============================================================================
JointGroupPositionController::JointGroupPositionController()
  : MultiInterfaceController(true)  // allow_optional_interfaces
{
  using hardware_interface::EffortJointInterface;
  using hardware_interface::PositionJointInterface;
  using hardware_interface::VelocityJointInterface;

  mAdapterFactory.registerFactory<
    PositionJointInterface, JointPositionAdapter>("position");
  mAdapterFactory.registerFactory<
    VelocityJointInterface, JointVelocityAdapter>("velocity");
  mAdapterFactory.registerFactory<
    EffortJointInterface, JointEffortAdapter>("effort");
}

//=============================================================================
JointGroupPositionController::~JointGroupPositionController()
{
}

//=============================================================================
bool JointGroupPositionController::init(
  hardware_interface::RobotHW *robot, ros::NodeHandle &n)
{
  using hardware_interface::JointStateInterface;

  // Build up the list of controlled DOFs.
  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  if (jointParameters.empty())
    return false;

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton)
    return false;

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton = getControlledMetaSkeleton(
    mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton)
    return false;

  // Retreive JointStateHandles required to update the position and velocity of
  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface)
  {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
    new SkeletonJointStateUpdater(mSkeleton, jointStateInterface));

  // Create adaptors to provide a uniform interface to different types.
  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mAdapters.resize(mControlledSkeleton->getNumDofs());

  for (size_t idof = 0; idof < numControlledDofs; ++idof)
  {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter)
      return false;

    ros::NodeHandle adapterNodeHandle = createDefaultAdapterNodeHandle(n, dof);
    if (!adapter->initialize(adapterNodeHandle)) return false;

    mAdapters[idof] = std::move(adapter);
  }

  // Initialize memory to hold the desired position to avoid allocations later.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredPosition.setZero();

  // Start command subscriber
  mCommandSubscriber = n.subscribe(
    "command", 1, &JointGroupPositionController::setCommand, this);

  return true;
}

//=============================================================================
void JointGroupPositionController::starting(const ros::Time& time)
{
  // Initialize the setpoint to the current joint positions.
  mSkeletonUpdater->update();
  mDesiredPosition = mControlledSkeleton->getPositions();
  mDesiredPositionBuffer.initRT(mDesiredPosition);

  ROS_INFO_STREAM("Initialized desired position to: "
    << mDesiredPosition.transpose());

  // Reset any internal state in the adapters (e.g. integral windup).
  for (const auto& adapter : mAdapters)
    adapter->reset();
}

//=============================================================================
void JointGroupPositionController::update(
  const ros::Time& time, const ros::Duration& period)
{
  const auto desiredVelocity = 0.;

  mDesiredPosition = *mDesiredPositionBuffer.readFromRT();

  // Compute inverse dynamics torques and store them in the skeleton. These
  // values may be queried by the adapters invoked below.
  mSkeletonUpdater->update();
  mSkeleton->computeInverseDynamics();

  // Delegate to the adapter for each joint. This directly writes to the
  // robot's hardware interface.
  for (size_t idof = 0; idof < mAdapters.size(); ++idof)
  {
    mAdapters[idof]->update(time, period,
      mControlledSkeleton->getPosition(idof), mDesiredPosition[idof],
      mControlledSkeleton->getVelocity(idof), desiredVelocity,
      mControlledSkeleton->getForce(idof));
  }
}

//=============================================================================
void JointGroupPositionController::setCommand(
  const sensor_msgs::JointState& msg)
{
  const auto numControlledDofs = mAdapters.size();

  if (msg.name.size() != numControlledDofs)
  {
    ROS_ERROR_STREAM("Received command with incorrect number of names:"
      << " expected " << numControlledDofs << ", got " << msg.name.size());
    return;
  }
  if (msg.position.size() != numControlledDofs)
  {
    ROS_ERROR_STREAM("Received command with incorrect number of positions:"
      << " expected " << numControlledDofs << ", got " << msg.position.size());
    return;
  }
  if (!msg.velocity.empty())
  {
    ROS_ERROR("Received command with velocities. Only position is supported.");
    return;
  }
  if (!msg.effort.empty())
  {
    ROS_ERROR("Received command with velocities. Only position is supported.");
    return;
  }

  Eigen::VectorXd desiredPosition(numControlledDofs);
  for (size_t i = 0; i < numControlledDofs; ++i)
  {
    const auto dof = mSkeleton->getDof(msg.name[i]);
    const auto dofIndex = mControlledSkeleton->getIndexOf(dof, false);
    if (dofIndex == dart::dynamics::INVALID_INDEX)
    {
      ROS_ERROR_STREAM("There is DegreeOfFreedom named '" << dof->getName()
        << "' under control.");
      return;
    }

    desiredPosition[dofIndex] = msg.position[i];
  }

  mDesiredPositionBuffer.writeFromNonRT(desiredPosition);
}

} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::JointGroupPositionController,
  controller_interface::ControllerBase)
