// TODO license/acknowlegement

#include <rewd_controllers/joint_group_position_controller.h>
#include <angles/angles.h>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_extension/KdlChainIdRne.h>
#include <pluginlib/class_list_macros.h>

using namespace rewd_controllers;

JointGroupPositionController::JointGroupPositionController()
  : loop_count_(0)
{}

JointGroupPositionController::~JointGroupPositionController()
{
  sub_command_.shutdown();
}

bool JointGroupPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::vector<std::string> joint_names;
  if (!n.getParam("joints", joint_names)) 
  {
    ROS_ERROR("No joints given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<sensor_msgs::JointState>("command", 1, &JointGroupPositionController::setCommand, this);

  // Get URDF
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  // Get joint handles and URDFS and save to maps
  for (std::vector<std::string>::iterator it = joint_names.begin(); it != joint_names.end(); ++it) {
    joints_[*it] = robot->getHandle(*it);
    joint_urdfs_[*it] = urdf.getJoint(*it);

    if (!joint_urdfs_[*it])
    {
        ROS_ERROR("Could not find joint '%s' in urdf", it->c_str());
        return false;
    }
  }

  KDL::Tree kdl_tree;
  kdl_tree_id.setTree(kdl_tree); // TODO is kdl_tree mutable?
  kdl_tree_id.loadFromParam("robot_description");

  return true;
}

// void JointGroupPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
// {
//   pid_controller_.setGains(p,i,d,i_max,i_min);
// }

// void JointGroupPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
// {
//   pid_controller_.getGains(p,i,d,i_max,i_min);
// }

// void JointGroupPositionController::printDebug()
// {
//   pid_controller_.printValues();
//   // TODO print kdl_tree info?
// }


// Set the joint positions command
// void JointGroupPositionController::setCommand(std::vector<std::string> names, std::vector<double> pos_commands)
// {
//   for (int i = 0; i < pos_commands.size(); ++i) {
//     command_structs_[names[i]].position_ = pos_commands[i];
//     command_structs_[names[i]].has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it
//   }

//   // the writeFromNonRT can be used in RT, if you have the guarantee that
//   //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
//   //  * there is only one single rt thread
//   commands_.writeFromNonRT(command_structs_);
// }

// // Set the joint position command with a velocity command as well
// void JointPositionController::setCommand(double pos_command, double vel_command)
// {
//   command_struct_.position_ = pos_command;
//   command_struct_.velocity_ = vel_command;
//   command_struct_.has_velocity_ = true;

//   command_.writeFromNonRT(command_struct_);
// }

void JointGroupPositionController::starting(const ros::Time& time)
{
  // TODO multiple joints
  // double pos_command = joint_.getPosition();

  // // Make sure joint is within limits if applicable
  // enforceJointLimits(pos_command);

  // command_struct_.position_ = pos_command;
  // command_struct_.has_velocity_ = false;

  // command_.initRT(command_struct_);

  // pid_controller_.reset();
}

void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  // TODO multiple joints
  
  // command_structs_ = *(commands_.readFromRT()); // TODO map of pointers?
  joint_state_command = *(command_buffer.readFromRT());

  const KDL::Tree& tree = kdl_tree_id.getTree(); // TODO 
  KDL::JntArray joint_positions = KDL::JntArray(tree.getNrOfJoints());
  // TODO use RosMsgConverters

  // std::map<std::string, KDL::TreeElement> segments = tree.getSegments();
  // for (std::map<std::string, KDL::TreeElement>::iterator it = segments.begin(); it != segments.end(); ++it) {
  //   // TODO set pos, vel, accl for every joint on kdl model
  //   const KDL::Joint& joint = it->second.segment.getJoint();
  //   joint_positions(
  // }

  
  // TODO get inverse dynamics
  // TODO 1. log combined, ID, and PID effort
  // TODO 2. add ID to PID effort
  // TODO double check herb model inertia w/ data sheet
  // TODO check torques needed by PID to keep arm still (inertia)

  // TODO combine with above loop?
  for (std::map<std::string, hardware_interface::JointHandle>::iterator it = joints_.begin();
         it != joints_.end(); ++it) {
    hardware_interface::JointHandle joint = it->second;

  //   // check if joint name is in message
  //   std::map<std::string, Commands>::const_iterator command_it = command_structs_.find(joint.getName());
  //   if (command_it == command_structs_.end()) {
  //     continue; // TODO right to skip joint if not in message?
  //   }
  //   Commands command_struct_ = command_it->second;

  //   double command_position = command_struct_.position_;
  //   double command_velocity = command_struct_.velocity_;
  //   bool has_velocity_ =  command_struct_.has_velocity_;

  //   double error, vel_error;
    double commanded_effort;

  //   double current_position = joint.getPosition();
  //   // joint_urdfs was populated from joints and already checked for missing joint names
  //   boost::shared_ptr<const urdf::Joint> joint_urdf_ = joint_urdfs_[joint.getName()];

  //   // Make sure joint is within limits if applicable
  //   // enforceJointLimits(command_position); // TODO

  //   // Compute position error
  //   if (joint_urdf_->type == urdf::Joint::REVOLUTE)
  //     {
  //       angles::shortest_angular_distance_with_limits(
  //                                                     current_position,
  //                                                     command_position,
  //                                                     joint_urdf_->limits->lower,
  //                                                     joint_urdf_->limits->upper,
  //                                                     error);
  //     }
  //   else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  //     {
  //       error = angles::shortest_angular_distance(current_position, command_position);
  //     }
  //   else //prismatic
  //     {
  //       error = command_position - current_position;
  //     }

  //   // Decide which of the two PID computeCommand() methods to call
  //   if (has_velocity_)
  //     {
  //       // Compute velocity error if a non-zero velocity command was given
  //       vel_error = command_velocity - joint.getVelocity();

  //       // Set the PID error and compute the PID command with nonuniform
  //       // time step size. This also allows the user to pass in a precomputed derivative error.
  //       commanded_effort = pid_controller_.computeCommand(error, vel_error, period);
  //     }
  //   else
  //     {
  //       // Set the PID error and compute the PID command with nonuniform
  //       // time step size.
  //       commanded_effort = pid_controller_.computeCommand(error, period);
  //     }

    joint.setCommand(commanded_effort); // TODO joints

  }
  // // publish state
  // if (loop_count_ % 10 == 0)
  //   {
  //     if(controller_state_publisher_ && controller_state_publisher_->trylock())
  //       {
  //         controller_state_publisher_->msg_.header.stamp = time;
  //         controller_state_publisher_->msg_.set_point = command_position;
  //         controller_state_publisher_->msg_.process_value = current_position;
  //         controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
  //         controller_state_publisher_->msg_.error = error;
  //         controller_state_publisher_->msg_.time_step = period.toSec();
  //         controller_state_publisher_->msg_.command = commanded_effort;

  //         double dummy;
  //         getGains(controller_state_publisher_->msg_.p,
  //                  controller_state_publisher_->msg_.i,
  //                  controller_state_publisher_->msg_.d,
  //                  controller_state_publisher_->msg_.i_clamp,
  //                  dummy);
  //         controller_state_publisher_->unlockAndPublish();
  //       }
  //   }
  loop_count_++;
}

void JointGroupPositionController::setCommand(const sensor_msgs::JointStateConstPtr& msg)
{
  command_buffer.writeFromNonRT(msg);
  // setCommands(msg->name, msg->position);
}

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
