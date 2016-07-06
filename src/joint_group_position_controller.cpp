#include <rewd_controllers/joint_group_position_controller.h>
#include <angles/angles.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>

namespace rewd_controllers {

using EffortJointInterface = hardware_interface::EffortJointInterface;
using JointStateInterface = hardware_interface::JointStateInterface;

JointGroupPositionController::JointGroupPositionController()
{
  // Static initialization code.
  mAdapterFactory.registerFactory<
    PositinoJointInterface, JointEffortAdapter>("position");
  mAdapterFactory.registerFactory<
    EffortJointInterface, JointEffortAdapter>("effort");
}

JointGroupPositionController::~JointGroupPositionController()
{
}

bool JointGroupPositionController::init(
  hardware_interface::RobotHW *robot, ros::NodeHandle &n)
{
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
    ROS_ERROR("Unable to get JointStateInterface from the RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
    new SkeletonJointStateUpdater(mSkeleton, jointStateInterface));

  // Create adaptors to provide a uniform interface to different types.
  mAdapters.resize(mControlledSkeleton->getNumDofs());

  for (size_t idof = 0; idof < mControlledSkeleton->getNumDofs(); ++idof)
  {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];
    mAdapters[idof] = mAdapterFactory.create(param.mType, robot);
  }

  // Initialize command struct vector sizes
  ROS_INFO("Allocating setpoint buffer");
  number_of_joints_ = controlled_skeleton_->getNumDofs();
  joint_state_command_.resize(number_of_joints_);

#if 0
  // Load PID Controllers using gains set on parameter server
  ROS_INFO("Initializing PID controllers");
  joint_pid_controllers_.resize(number_of_joints_);
  for (size_t i = 0; i < number_of_joints_; ++i) {
    std::string const &dof_name = controlled_skeleton_->getDof(i)->getName();
    ros::NodeHandle pid_nh(n, std::string("gains/") + dof_name);
    if (!joint_pid_controllers_[i].init(pid_nh)) {
      return false;
    }
  }
#endif

  // Start command subscriber
  mCommandSubscriber = n.subscribe(
    "command", 1, &JointGroupPositionController::setCommand, this);

  ROS_INFO("JointGroupPositionController initialized successfully");
  return true;
}

void JointGroupPositionController::starting(const ros::Time& time)
{
  for (size_t i = 0; i < number_of_joints_; ++i)
  {
    joint_state_command_[i] = controlled_joint_handles_[i].getPosition();
    joint_pid_controllers_[i].reset();
  }

  command_buffer_.initRT(joint_state_command_);
}

void JointGroupPositionController::update(
  const ros::Time& time, const ros::Duration& period)
{
  joint_state_command_ = *(command_buffer_.readFromRT());

  // Update mSkeleton with the latest position and velocity measurements.
  mSkeletonUpdater->update();
  mSkeleton->computeInverseDynamics();



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
}

void JointGroupPositionController::setCommand(const sensor_msgs::JointState& msg) {
  if (msg.name.size() != number_of_joints_
      || msg.position.size() != number_of_joints_) {
    ROS_ERROR("Number of joint names specified in JointState message [%d] does not match number of controlled joints [%d]", msg.name.size(), number_of_joints_);
    return;
  }

  if (msg.position.size() != number_of_joints_) {
    ROS_ERROR("Number of joint positions specified in JointState message [%d] does not match number of controlled joints [%d]", msg.position.size(), number_of_joints_);
    return;
  }

  std::vector<double> position_command(controlled_skeleton_->getNumDofs());

  for (size_t i = 0; i < number_of_joints_; ++i) {
    std::string const &joint_name = msg.name[i];

    auto const it = controlled_joint_map_.find(joint_name);
    if (it == std::end(controlled_joint_map_)) {
      ROS_ERROR("Unknown joint '%s' in message at index %d.",
        joint_name.c_str(), i);
      continue;
    }

    position_command[i] = msg.position[i];
  }

  command_buffer_.writeFromNonRT(position_command);
}

} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::JointGroupPositionController,
  controller_interface::ControllerBase)
