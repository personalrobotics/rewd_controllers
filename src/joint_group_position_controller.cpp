// TODO license/acknowlegement

#include <rewd_controllers/joint_group_position_controller.h>
#include <rewd_controllers/RosMsgConverter.h>
#include <angles/angles.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <dart/dynamics/dynamics.h>
#include <dart/utils/urdf/DartLoader.h>
#include <r3/util/CatkinResourceRetriever.h>

namespace rewd_controllers {

JointGroupPositionController::JointGroupPositionController()
{
}

JointGroupPositionController::~JointGroupPositionController()
{
}

bool JointGroupPositionController::init(
  hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Load the URDF XML from the parameter server.
  std::string robot_description_parameter;
  n.param<std::string>("robot_description_parameter",
    robot_description_parameter, "/robot_description");

  std::string robot_description;
  if (!n.getParam(robot_description_parameter, robot_description)) {
    ROS_ERROR("Failed loading URDF from '%s' parameter.",
      robot_description_parameter.c_str());
    return false;
  }

  // Load the URDF as a DART model.
  auto const resource_retriever
    = std::make_shared<r3::util::CatkinResourceRetriever>();
  dart::common::Uri const base_uri;

  ROS_INFO("Loading DART model from URDF...");
  dart::utils::DartLoader urdf_loader;
  skeleton_ = urdf_loader.parseSkeletonString(
    robot_description, base_uri, resource_retriever);
  if (!skeleton_) {
    ROS_ERROR("Failed loading URDF as a DART Skeleton.");
    return false;
  }
  ROS_INFO("Loading DART model from URDF...DONE");

  // Build up the list of controlled DOFs.
  ROS_INFO("Getting joint names");
  std::vector<std::string> dof_names;
  if (!n.getParam("joints", dof_names)) {
    ROS_ERROR("Unable to read controlled DOFs from the parameter '%s/joints'.",
      n.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Creating controlled Skeleton");
  controlled_skeleton_ = dart::dynamics::Group::create("controlled");
  for (std::string const &dof_name : dof_names) {
    dart::dynamics::DegreeOfFreedom *const dof = skeleton_->getDof(dof_name);
    if (!dof) {
      ROS_ERROR("There is no DOF named '%s'.", dof_name.c_str());
      return false;
    }
    controlled_skeleton_->addDof(dof, true);

    controlled_joint_map_.emplace(
      dof->getName(), controlled_skeleton_->getNumDofs() - 1);
  }

  // Get all joint handles.
  ROS_INFO("Getting controlled JointHandles");
  controlled_joint_handles_.reserve(controlled_skeleton_->getNumDofs());
  for (dart::dynamics::DegreeOfFreedom const *const dof : controlled_skeleton_->getDofs()) {
    std::string const &dof_name = dof->getName();
    hardware_interface::JointHandle handle;

    try {
      handle = robot->getHandle(dof_name);
    } catch (hardware_interface::HardwareInterfaceException const &e) {
      ROS_ERROR("Failed getting JointHandle for controlled DOF '%s'.", dof_name.c_str());
      return false;
    }

    controlled_joint_handles_.push_back(handle);
  }

  ROS_INFO("Getting all JointHandles");
  joint_handles_.reserve(skeleton_->getNumDofs());
  for (dart::dynamics::DegreeOfFreedom const *const dof : skeleton_->getDofs()) {
    std::string const &dof_name = dof->getName();
    hardware_interface::JointHandle handle;

    try {
      handle = robot->getHandle(dof_name);
    } catch (hardware_interface::HardwareInterfaceException const &e) {
      ROS_WARN("Failed getting JointHandle for DOF '%s'.", dof_name.c_str());
      continue;
    }

    joint_handles_.push_back(handle);
  }

  // Initialize command struct vector sizes
  ROS_INFO("Allocating setpoint buffer");
  number_of_joints = controlled_skeleton_->getNumDofs();
  joint_state_command.resize(number_of_joints);

  // Load PID Controllers using gains set on parameter server
  ROS_INFO("Initializing PID controllers");
  joint_pid_controllers.resize(number_of_joints);
  for (size_t i = 0; i < number_of_joints; ++i) {
    std::string const &dof_name = controlled_skeleton_->getDof(i)->getName();
    ros::NodeHandle pid_nh(n, std::string("gains/") + dof_name);
    if (!joint_pid_controllers[i].init(pid_nh)) {
      return false;
    }
  }

  // Start command subscriber
  command_sub = n.subscribe("command", 1, &JointGroupPositionController::setCommand, this);

  ROS_INFO("JointGroupPositionController initialized successfully");
  return true;
}

void JointGroupPositionController::starting(const ros::Time& time) {
  for (size_t i = 0; i < number_of_joints; ++i) {
    joint_state_command[i] = controlled_joint_handles_[i].getPosition();
    joint_pid_controllers[i].reset();
  }

  command_buffer.initRT(joint_state_command);
  ROS_INFO("JointGroupPositionController started successfully");
}

void JointGroupPositionController::update(
  const ros::Time& time, const ros::Duration& period)
{
  joint_state_command = *(command_buffer.readFromRT());

  for (hardware_interface::JointHandle &handle : joint_handles_) {
    dart::dynamics::DegreeOfFreedom *const dof
      = skeleton_->getDof(handle.getName());
    if (!dof)
      continue; // This should never happen.

    dof->setPosition(handle.getPosition());
    dof->setVelocity(handle.getVelocity());
    dof->setAcceleration(0.);
  }

  skeleton_->computeInverseDynamics();

  // PID control of each joint
  for (size_t i = 0; i < number_of_joints; ++i) {
    dart::dynamics::DegreeOfFreedom *const dof
      = controlled_skeleton_->getDof(i);
    dart::dynamics::Joint *const joint = dof->getJoint();

    double const position_desired = joint_state_command[i];
    double const position_actual = dof->getPosition();

    // TODO: Make sure joint is within limits if applicable
    // enforceJointLimits(joint_urdf, command_position);

    // Compute position error
    double position_error;
    if (joint->getType() == dart::dynamics::RevoluteJoint::getStaticType()) {
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
    } else if (joint->getType() == dart::dynamics::PrismaticJoint::getStaticType()) {
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
    double const effort_pid = joint_pid_controllers[i].computeCommand(
      position_error, period);
    double const effort_command = effort_pid; // TODO incorporate ID.

    hardware_interface::JointHandle &joint_handle
      = controlled_joint_handles_[i];
    joint_handle.setCommand(effort_command);
  }
}

void JointGroupPositionController::setCommand(const sensor_msgs::JointState& msg) {
  if (msg.name.size() != number_of_joints
      || msg.position.size() != number_of_joints) {
    ROS_ERROR("Number of joint names specified in JointState message [%d] does not match number of controlled joints [%d]", msg.name.size(), number_of_joints);
    return;
  }

  if (msg.position.size() != number_of_joints) {
    ROS_ERROR("Number of joint positions specified in JointState message [%d] does not match number of controlled joints [%d]", msg.position.size(), number_of_joints);
    return;
  }

  std::vector<double> position_command(controlled_skeleton_->getNumDofs());

  for (size_t i = 0; i < number_of_joints; ++i) {
    std::string const &joint_name = msg.name[i];

    auto const it = controlled_joint_map_.find(joint_name);
    if (it == std::end(controlled_joint_map_)) {
      ROS_ERROR("Unknown joint '%s' in message at index %d.",
        joint_name.c_str(), i);
      continue;
    }

    position_command[i] = msg.position[i];
  }

  command_buffer.writeFromNonRT(position_command);
}


// TODO verify the fix to angles obsoletes this function
// // Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
// void JointGroupPositionController::enforceJointLimits(double &command)
// {
//   // Check that this joint has applicable limits
//   if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
//   {
//     if( command > joint_urdf_->limits->upper ) // above upper limnit
//     {
//       command = joint_urdf_->limits->upper;
//     }
//     else if( command < joint_urdf_->limits->lower ) // below lower limit
//     {
//       command = joint_urdf_->limits->lower;
//     }
//   }
// }


} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::JointGroupPositionController,
  controller_interface::ControllerBase)

