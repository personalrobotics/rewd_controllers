#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pluginlib/class_list_macros.h>
#include <rewd_controllers/joint_group_position_controller.h>
//#include <angles/angles.h>
//#include <hardware_interface/hardware_interface.h>

namespace rewd_controllers {

//=============================================================================
JointGroupPositionController::JointGroupPositionController()
{
  using hardware_interface::EffortJointInterface;
  using hardware_interface::PositionJointInterface;

  mAdapterFactory.registerFactory<
    PositionJointInterface, JointPositionAdapter>("position");
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
  const ros::NodeHandle gainsNodeHandle{n, "gains"};
  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mAdapters.resize(mControlledSkeleton->getNumDofs());

  for (size_t idof = 0; idof < numControlledDofs; ++idof)
  {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter)
      return false;

    // Initialize the adapter using parameters stored on the parameter server.
    ros::NodeHandle adapterNodeHandle{gainsNodeHandle, dof->getName()};
    if (!adapter->initialize(adapterNodeHandle))
      return false;

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
  mDesiredPositionBuffer.initRT(mSkeleton->getPositions());

  // Reset any internal state in the adapters (e.g. integral windup).
  for (const auto& adapter : mAdapters)
    adapter->reset();
}

//=============================================================================
void JointGroupPositionController::update(
  const ros::Time& time, const ros::Duration& period)
{
  mDesiredPosition = *mDesiredPositionBuffer.readFromRT();

  // Compute inverse dynamics torques and store them in the skeleton. These
  // values may be queried by the adapters invoked below.
  mSkeletonUpdater->update();
  mSkeleton->computeInverseDynamics();

  // Delegate to the adapter for each joint. This directly writes to the
  // robot's hardware interface.
  for (size_t idof = 0; idof < mAdapters.size(); ++idof)
    mAdapters[idof]->update(time, period, mDesiredPosition[idof]);
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

#if 0
  // PID control of each joint
  for (size_t i = 0; i < number_of_joints_; ++i) {
    dart::dynamics::DegreeOfFreedom *const dof
      = controlled_skeleton_->getDof(i);
    dart::dynamics::Joint *const joint = dof->getJoint();

    double const position_desired = joint_state_command_[i];
    double const position_actual = dof->getPosition();

    // TODO: Make sure joint is within limits if applicable
    // enforceJointLimits(joint_urdf, command_position);

    // Compute position error
    double position_error;
    if (&joint->getType() == &dart::dynamics::RevoluteJoint::getStaticType()) {
      if (dof->isCyclic()) {
        position_error = angles::shortest_angular_distance(
          position_desired, position_actual);
      } else {
        position_error = position_desired - position_actual;
       // TODO use this when add enforceJointLimits
#if 0
       angles::shortest_angular_distance_with_limits(current_position,
                                                     command_position,
                                                     joint_urdf->limits->lower,
                                                     joint_urdf->limits->upper,
                                                     error);
#endif
      }
    } else if (&joint->getType() == &dart::dynamics::PrismaticJoint::getStaticType()) {
      position_error = position_desired - position_actual;
    } else {
      position_error = 0.;
      ROS_ERROR(
        "DegreeOfFreedom '%s' is from joint '%s' with unknown type '%s'.",
        dof->getName().c_str(), joint->getName().c_str(),
        joint->getType().c_str());
    }

    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    double const effort_inversedynamics = dof->getForce();
    double const effort_pid = joint_pid_controllers_[i].computeCommand(
      position_error, period);
    double const effort_command = effort_pid; // TODO incorporate ID.

    hardware_interface::JointHandle &joint_handle
      = controlled_joint_handles_[i];
    joint_handle.setCommand(effort_command);
  }
#endif

} // namespace rewd_controllers

//=============================================================================
PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::JointGroupPositionController,
  controller_interface::ControllerBase)
