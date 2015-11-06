// TODO license/acknowlegement

#include <rewd_controllers/gravity_compensation_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <dart/dynamics/dynamics.h>
#include <dart/utils/urdf/DartLoader.h>
#include <r3/util/CatkinResourceRetriever.h>

namespace rewd_controllers {

GravityCompensationController::GravityCompensationController() {}

GravityCompensationController::~GravityCompensationController() {}

bool GravityCompensationController::init(
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

  // Initialize logging
  ROS_INFO("Opening log file");
  logfile.open("rewd_commands.log");
  if (!logfile.is_open()) {
    ROS_ERROR("Failed to open logfile");
    return false;
  }

  ROS_INFO("GravityCompensationController initialized successfully");
  return true;
}

void GravityCompensationController::update(
  const ros::Time& time, const ros::Duration& period)
{
  // joint_state_command = *(command_buffer.readFromRT());

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

  for (size_t i = 0; i < controlled_skeleton_->getNumDofs(); ++i) {
    dart::dynamics::DegreeOfFreedom *const dof
      = controlled_skeleton_->getDof(i);

    double const effort_inversedynamics = dof->getForce();

    hardware_interface::JointHandle &joint_handle
      = controlled_joint_handles_[i];
    joint_handle.setCommand(effort_inversedynamics);

    logfile << "Joint [" << dof->getName() << "]: ID = " << effort_inversedynamics << "\n";
  }
}

} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(
  rewd_controllers::GravityCompensationController,
  controller_interface::ControllerBase)

