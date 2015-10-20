// TODO license/acknowlegement

#include <rewd_controllers/joint_group_position_controller.h>
#include <rewd_controllers/RosMsgConverter.h>
#include <angles/angles.h>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/tree.hpp>
#include <kdl_extension/JointDynamicsData.h>
#include <kdl_extension/KdlChainIdRne.h>
#include <pluginlib/class_list_macros.h>

using namespace rewd_controllers;

JointGroupPositionController::JointGroupPositionController() {}

JointGroupPositionController::~JointGroupPositionController() {
  command_sub.shutdown();
}

bool JointGroupPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
  // Get joint name from parameter server
  if (!n.getParam("joints", joint_names)) {
    ROS_ERROR("No joints given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  std::set<std::string> joint_names_wo_dups(joint_names.begin(), joint_names.end());
  if (joint_names_wo_dups.size() != joint_names.size()) {
    ROS_ERROR("Duplicate joint names were detected:");
    for (int i = 0; i < joint_names.size(); ++i) {
      ROS_ERROR("%s", joint_names[i].c_str());
    }
    return false;
  }

  number_of_joints = joint_names.size();
  if (!number_of_joints) {
    ROS_ERROR("No joints specified in 'joints' parameter");
    return false;
  }

  // Initialize command struct vector sizes
  joint_state_command.reserve(number_of_joints);

  // Load PID Controllers using gains set on parameter server
  joint_pid_controllers.resize(number_of_joints);
  for (unsigned int i = 0; i < number_of_joints; ++i) {
    ros::NodeHandle pid_nh(n, std::string("gains/") + joint_names[i]);
    if (!joint_pid_controllers[i].init(pid_nh)) {
      return false;
    }
  }

  // Get URDF
  urdf::Model urdf;
  // TODO load "robot_description" string from parameter itself?
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR("Failed to load 'robot_description' parameter");
    return false;
  }

  // Get joint handles and URDFS and save to maps
  joints.resize(number_of_joints);
  joint_urdfs.resize(number_of_joints);
  for (unsigned int i = 0; i < number_of_joints; ++i) {
    joints[i] = robot->getHandle(joint_names[i]);
    joint_urdfs[i] = urdf.getJoint(joint_names[i]);

    if (!joint_urdfs[i]) {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
        return false;
    }
  }

  // Determine beginning and end links
  std::string base_link_name = joint_urdfs[0]->parent_link_name;
  std::string tool_link_name = joint_urdfs[number_of_joints - 1]->parent_link_name;

  kdl_tree_id.setTree(kdl_tree);
  kdl_tree_id.loadFromParam("robot_description");

  // Start command subscriber
  command_sub = n.subscribe("command", 1, &JointGroupPositionController::setCommand, this);

  return true;
}

void JointGroupPositionController::starting(const ros::Time& time) {
  for (unsigned int i = 0; i < number_of_joints; ++i) {
    joint_state_command[i] = joints[i].getPosition();
    joint_pid_controllers[i].reset();
  }

  command_buffer.initRT(joint_state_command);
}

void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period) {
  // TODO find error/log/debug solution for realtime

  joint_state_command = *(command_buffer.readFromRT());

  kdl_extension::JointDynamicsData jd;
  jd.InitializeMaps(kdl_tree);
  KDL::Twist v_in; // TODO from base
  KDL::Twist a_in; // TODO from base
  KDL::Wrench f_out;
  KDL::RigidBodyInertia I_out;
  kdl_tree_id.treeRecursiveNewtonEuler(jd, "null", "null", v_in, a_in, f_out, I_out);

  ROS_DEBUG("INVERSE DYNAMICS tx=%d ty=%d tz=%d", f_out[3], f_out[4], f_out[5]);

  // TODO get inverse dynamics
  // TODO 1. log combined, ID, and PID effort
  // TODO 2. add ID to PID effort
  // TODO double check herb model inertia w/ data sheet
  // TODO check torques needed by PID to keep arm still (inertia)


  for(unsigned int i = 0; i < number_of_joints; ++i) {
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
        angles::shortest_angular_distance_with_limits(current_position,
                                                      command_position,
                                                      joint_urdf->limits->lower,
                                                      joint_urdf->limits->upper,
                                                      error);
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

    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    effort_command = joint_pid_controllers[i].computeCommand(error, period);

    joint.setCommand(effort_command);
    ROS_DEBUG("PID EFFORT COMMAND: %s = %d", joint_names[i].c_str(), effort_command);
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

  for (unsigned int i = 0; i < number_of_joints; ++i) {
    for (unsigned int k = 0; k < number_of_joints; ++k) {
      if (joint_names[k] == msg.name[i]) {
        joint_state_command[k] = msg.position[i];
      }
      else if (i == k) {
        ROS_ERROR("Unknown joint in JointState message: '%s'", msg.name[i].c_str());
      }
    }
  }

  command_buffer.writeFromNonRT(joint_state_command);
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


PLUGINLIB_EXPORT_CLASS( rewd_controllers::JointGroupPositionController, controller_interface::ControllerBase)
