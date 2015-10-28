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

  dart::utils::DartLoader urdf_loader;
  skeleton_ = urdf_loader.parseSkeletonString(
    robot_description, base_uri, resource_retriever);
  if (!skeleton_) {
    ROS_ERROR("Failed loading URDF as a DART Skeleton.");
    return false;
  }

  // Build up the list of controlled DOFs.
  std::vector<std::string> dof_names;
  if (!n.getParam("joints", dof_names)) {
    ROS_ERROR("Unable to read controlled DOFs from the parameter '%s/joints'.",
      n.getNamespace().c_str());
    return false;
  }

  controlled_skeleton_ = dart::dynamics::Group::create("controlled");
  for (std::string const &dof_name : dof_names) {
    dart::dynamics::DegreeOfFreedom *const dof = skeleton_->getDof(dof_name);
    if (!dof) {
      ROS_ERROR("There is no DOF named '%s'.", dof_name.c_str());
      return false;
    }
    controlled_skeleton_->addDof(dof, true);
  }

  // Initialize command struct vector sizes
  number_of_joints = controlled_skeleton_->getNumDofs();
  joint_state_command.resize(number_of_joints);

  // Load PID Controllers using gains set on parameter server
  joint_pid_controllers.resize(number_of_joints);
  for (size_t i = 0; i < number_of_joints; ++i) {
    ros::NodeHandle pid_nh(n, std::string("gains/") + joint_names[i]);
    if (!joint_pid_controllers[i].init(pid_nh)) {
      return false;
    }
  }

  // Initialize logging
  logfile.open("rewd_commands.log");
  if (!logfile.is_open()) {
    ROS_ERROR("Failed to open logfile");
    return false;
  }

  // Start command subscriber
  command_sub = n.subscribe("command", 1, &JointGroupPositionController::setCommand, this);

  ROS_INFO("JointGroupPositionController initialized successfully");
  return true;
}

void JointGroupPositionController::starting(const ros::Time& time) {
  for (size_t i = 0; i < number_of_joints; ++i) {
    joint_state_command[i] = joints[i].getPosition();
    joint_pid_controllers[i].reset();
  }

  command_buffer.initRT(joint_state_command);
  ROS_INFO("JointGroupPositionController started successfully");
}

void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period) {
  joint_state_command = *(command_buffer.readFromRT());

  // PID control of each joint
  for(size_t i = 0; i < number_of_joints; ++i) {
    double command_position = joint_state_command[i];
    double error;
    double effort_command;
    hardware_interface::JointHandle& joint = joints[i];
    boost::shared_ptr<const urdf::Joint>& joint_urdf = joint_urdfs[i];
    double current_position = joint.getPosition();

    // TODO removed below?
    // Make sure joint is within limits if applicable
    // enforceJointLimits(joint_urdf, command_position); // TODO


    // Compute position error
    if (joint_urdf->type == urdf::Joint::REVOLUTE) {
        error = command_position - current_position;
       // TODO use this when add enforceJointLimits
       // angles::shortest_angular_distance_with_limits(current_position,
       //                                               command_position,
       //                                               joint_urdf->limits->lower,
       //                                               joint_urdf->limits->upper,
       //                                               error);
    }
    else if (joint_urdf->type == urdf::Joint::CONTINUOUS) {
        error = angles::shortest_angular_distance(current_position, command_position);
    }
    else if (joint_urdf->type == urdf::Joint::PRISMATIC) {
        error = command_position - current_position;
    }
    else {
      ROS_ERROR("Unknown joint type");
      continue;
    }

    // TODO rm

    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    effort_command = joint_pid_controllers[i].computeCommand(error, period);

    // TODO incorporate ID
    joint.setCommand(effort_command);
    // std::cout << "joint: " << joint_names[i] << "\ncurrent_position: " << current_position << "\ncommand_position: " 
    //           << command_position << "\nerror: " << error << "\nlimit_upper: " << joint_urdf->limits->upper << "\nlimit_lower: " << joint_urdf->limits->lower << std::endl;

    logfile << "PID Effort Command: " << joint_names[i] << " = " << effort_command << "\n";
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

  Command position_command;
  position_command.resize(number_of_joints);
  for (size_t i = 0; i < number_of_joints; ++i) {
    for (size_t k = 0; k < number_of_joints; ++k) {
      if (joint_names[k] == msg.name[i]) {
        position_command[k] = msg.position[i];
      }
      else if (i == k) {
        ROS_ERROR("Unknown joint in JointState message: '%s'", msg.name[i].c_str());
      }
    }
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

