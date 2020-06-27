#include <rewd_controllers/MoveUntilTouchCartVelocityController.hpp>

#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

#include <pr_control_msgs/SetCartesianVelocityAction.h>
#include <pr_hardware_interfaces/CartesianVelocityInterface.h>

#define SE3_SIZE 6

namespace rewd_controllers {
namespace {
//=============================================================================
std::vector<double> toVector(const Eigen::VectorXd &input) {
  return std::vector<double>{input.data(), input.data() + input.size()};
}

} // namespace

//=============================================================================
MoveUntilTouchCartVelocityController::MoveUntilTouchCartVelocityController()
    : MultiInterfaceController(true) // allow_optional_interfaces
{}

//=============================================================================
MoveUntilTouchCartVelocityController::~MoveUntilTouchCartVelocityController() {}

//=============================================================================
bool MoveUntilTouchCartVelocityController::init(
    hardware_interface::RobotHW *robot, ros::NodeHandle &n) {
  using hardware_interface::JointModeInterface;
  using hardware_interface::JointStateInterface;
  using pr_hardware_interfaces::CartesianVelocityInterface;

  // Load the URDF as a Skeleton.
  mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton)
    return false;

  // Have skeleton update from joint state interface
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface) {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
      new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  // Load control interfaces and handles
  const auto jointModeInterface = robot->get<JointModeInterface>();
  if (!jointModeInterface) {
    ROS_ERROR("Unable to get JointModeInterface from RobotHW instance.");
    return false;
  }

  try {
    mJointModeHandle = jointModeInterface->getHandle("joint_mode");
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Unable to get joint mode interface for robot");
    return false;
  }

  const auto cartVelInterface = robot->get<CartesianVelocityInterface>();
  if (!cartVelInterface) {
    ROS_ERROR(
        "Unable to get CartesianVelocityInterface from RobotHW instance.");
    return false;
  }

  try {
    mCartVelHandle = cartVelInterface->getHandle("cart_vel");
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Unable to get cartesian velocity interface for robot.");
    return false;
  }

  // init local vars
  mCurrentCartVel.set(nullptr);

  // TODO: Initialize FT sensor

  // Start the action server. This must be last.
  using std::placeholders::_1;
  mActionServer.reset(new actionlib::ActionServer<Action>{
      n, "cart_velocity",
      std::bind(&MoveUntilTouchCartVelocityController::goalCallback, this, _1),
      std::bind(&MoveUntilTouchCartVelocityController::cancelCallback, this,
                _1),
      false});
  mActionServer->start();

  ROS_INFO("MoveUntilTouchCartVelocityController initialized successfully");
  return true;
}

//=============================================================================
void MoveUntilTouchCartVelocityController::starting(const ros::Time &time) {
  // Set Joint Mode to Other
  lastMode = mJointModeHandle.getMode();
  mJointModeHandle.setMode(JointModes::NOMODE);
}

//=============================================================================
void MoveUntilTouchCartVelocityController::stopping(const ros::Time &time) {
  // Return joint mode to what it was before
  mJointModeHandle.setMode(lastMode);
}

//=============================================================================
void MoveUntilTouchCartVelocityController::update(const ros::Time &time,
                                                  const ros::Duration &period) {
  mSkeletonUpdater->update();

  Eigen::VectorXd setVelocity;
  setVelocity.resize(SE3_SIZE);
  setVelocity.setZero();

  // Load current command
  std::shared_ptr<CartVelContext> context;
  mCurrentCartVel.get(context);

  if (context && !context->mCompleted.load()) {
    // check duration
    ros::Duration runTime = time - context->mStartTime;
    if (runTime >= context->mDuration) {
      // Successful run
      Result result;
      result.error_code = Result::SUCCESSFUL;
      result.error_string = "";
      context->mGoalHandle.setSucceeded(result);
      context->mCompleted.store(true);
    }
    // TODO: check FT sensor
    else {
      setVelocity = context->mDesiredVelocity;
    }
  }

  // Execute velocity command
  mCartVelHandle.setCommand(toVector(setVelocity));
}

//=============================================================================
void MoveUntilTouchCartVelocityController::goalCallback(GoalHandle goalHandle) {
  const auto goal = goalHandle.getGoal();
  ROS_INFO_STREAM("Received cartesian velocity '" << goalHandle.getGoalID().id
                                                  << "'.");

  // Setup the new trajectory.
  const auto newContext = std::make_shared<CartVelContext>();
  newContext->mStartTime = ros::Time::now();
  newContext->mGoalHandle = goalHandle;
  newContext->mDuration = goal->exec_time;
  newContext->mDesiredVelocity.resize(SE3_SIZE);
  newContext->mCompleted.store(false);

  // Copy over velocity data
  newContext->mDesiredVelocity(0) = goal->command.linear.x;
  newContext->mDesiredVelocity(1) = goal->command.linear.y;
  newContext->mDesiredVelocity(2) = goal->command.linear.z;
  newContext->mDesiredVelocity(3) = goal->command.angular.x;
  newContext->mDesiredVelocity(4) = goal->command.angular.y;
  newContext->mDesiredVelocity(5) = goal->command.angular.z;

  // TODO: add velocity checks

  // We've accepted the goal
  newContext->mGoalHandle.setAccepted();
  mCurrentCartVel.set(newContext);
}

//=============================================================================
void MoveUntilTouchCartVelocityController::cancelCallback(
    GoalHandle goalHandle) {
  ROS_INFO_STREAM("Requesting cancelation of velocity '"
                  << goalHandle.getGoalID().id << "'.");
  goalHandle.setAccepted();
  mCurrentCartVel.set(nullptr);
}

} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(rewd_controllers::MoveUntilTouchCartVelocityController,
                       controller_interface::ControllerBase)
