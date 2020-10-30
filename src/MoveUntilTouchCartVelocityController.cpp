#include <rewd_controllers/MoveUntilTouchCartVelocityController.hpp>

#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>

#include <pr_control_msgs/SetCartesianVelocityAction.h>

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
{
  mUseFT = false;
}

//=============================================================================
MoveUntilTouchCartVelocityController::~MoveUntilTouchCartVelocityController() {}

//=============================================================================
bool MoveUntilTouchCartVelocityController::init(
    hardware_interface::RobotHW *robot, ros::NodeHandle &n) {
  using hardware_interface::JointModeInterface;
  using hardware_interface::JointStateInterface;
  using hardware_interface::VelocityJointInterface;

  // Enable force/torque functionality if requested
  mUseFT = false;
  n.getParam("enableFT", mUseFT);

  if(mUseFT) {
    // load name of force/torque sensor handle from paramter
    std::string ft_wrench_name;
    if (!n.getParam("forcetorque_wrench_name", ft_wrench_name)) {
      ROS_ERROR("Failed to load 'forcetorque_wrench_name' parameter.");
      return false;
    }
    // load name of force/torque tare handle from paramter
    std::string ft_tare_name;
    if (!n.getParam("forcetorque_tare_name", ft_tare_name)) {
      ROS_ERROR("Failed to load 'forcetorque_tare_name' parameter.");
      return false;
    }

    // load force/torque saturation limits from parameter
    double forceLimit = 0.0;
    if (!n.getParam("sensor_force_limit", forceLimit)) {
      ROS_ERROR("Failed to load 'sensor_force_limit' parameter.");
      return false;
    }
    double torqueLimit = 0.0;
    if (!n.getParam("sensor_torque_limit", torqueLimit)) {
      ROS_ERROR("Failed to load 'sensor_torque_limit' parameter.");
      return false;
    }
    if (forceLimit < 0) {
      ROS_ERROR("sensor_force_limit must be positive or zero");
      return false;
    }
    if (torqueLimit < 0) {
      ROS_ERROR("sensor_torque_limit must be positive or zero");
      return false;
    }

    // Init FT Threshold Server
    mFTThresholdServer.reset(new FTThresholdServer{n,
            ft_wrench_name,
            ft_tare_name,
            forceLimit,
            torqueLimit});
  }

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

  // Build up the list of controlled joints.
  const auto jointParameters = loadJointsFromParameter(n, "joints", "velocity");
  if (jointParameters.empty())
    return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto &param : jointParameters) {
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

    if (param.mType != "velocity") {
      ROS_ERROR_STREAM("Joint '"
                       << param.mName
                       << "' is not velocity-controlled and cannot be "
                          "used in a cartesian velocity controller");
      return false;
    }
  }

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton =
      getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton) {
    return false;
  }

  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mControlledJointHandles.resize(numControlledDofs);

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
  const auto velocityJointInterface = robot->get<VelocityJointInterface>();
  if (!velocityJointInterface) {
    ROS_ERROR("Unable to get VelocityJointInterface from RobotHW instance.");
    return false;
  }

  for (size_t idof = 0; idof < numControlledDofs; ++idof) {
    const auto dofName = mControlledSkeleton->getDof(idof)->getName();
    try {
      auto handle = velocityJointInterface->getHandle(dofName);
      mControlledJointHandles[idof] = handle;
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM(
          "Unable to get interface of type 'VelocityJointInterface' for joint '"
          << dofName << "'.");
      return false;
    }
  }

  // init local vars
  mCurrentCartVel.set(nullptr);
  if (!n.getParam("ee", mEEName)) {
    ROS_ERROR_STREAM("Parameter '" << n.getNamespace()
                                   << "/ee' is required.");
  }

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
  mJointModeHandle.setMode(JointModes::MODE_VELOCITY);

  // start FTThresholdServer
  mFTThresholdServer->start();
}

//=============================================================================
void MoveUntilTouchCartVelocityController::stopping(const ros::Time &time) {
  // Return joint mode to what it was before
  mJointModeHandle.setMode(lastMode);

  // stop FTThresholdServer
  mFTThresholdServer->stop();
}

//=============================================================================
void MoveUntilTouchCartVelocityController::update(const ros::Time &time,
                                                  const ros::Duration &period) {
  mSkeletonUpdater->update();

  Eigen::VectorXd setVelocity;
  setVelocity.resize(SE3_SIZE);
  setVelocity.setZero();

  // Check FT Threshold
  bool ftThresholdTripped = false;
  std::string message = "";
  if(mUseFT) {
    ftThresholdTripped = mFTThresholdServer->shouldStopExecution(message);
  }

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
    } else if (ftThresholdTripped) {
      // Aborted run
      Result result;
      result.error_code = Result::FORCE_THRESH;
      result.error_string = message;
      context->mGoalHandle.setAborted(result);
      context->mCompleted.store(true);
    } else {
      setVelocity = context->mDesiredVelocity;
    }
  }

  // Execute velocity command
  /* Get full (with orientation) Jacobian of our end-effector */
  Eigen::VectorXd jointVels(mControlledSkeleton->getNumDofs());
  jointVels.setZero();
  if (setVelocity.norm() != 0) {
    Eigen::MatrixXd J = mSkeleton->getBodyNode(mEEName)->getWorldJacobian();
    Eigen::MatrixXd JtJ = J.transpose() * J;
    if (JtJ.determinant() != 0) {
      jointVels = JtJ.inverse() * J.transpose() * setVelocity;
    } else {
      ROS_ERROR_STREAM("Error: at singularity. Cartesian control impossible.");
    }
  }

  for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof) {
    auto jointHandle = mControlledJointHandles[idof];
    jointHandle.setCommand(jointVels(idof));
  }
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
  newContext->mDesiredVelocity(0) = goal->command.angular.x;
  newContext->mDesiredVelocity(1) = goal->command.angular.y;
  newContext->mDesiredVelocity(2) = goal->command.angular.z;
  newContext->mDesiredVelocity(3) = goal->command.linear.x;
  newContext->mDesiredVelocity(4) = goal->command.linear.y;
  newContext->mDesiredVelocity(5) = goal->command.linear.z;

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
  mCurrentCartVel.set(nullptr);
}

} // namespace rewd_controllers

PLUGINLIB_EXPORT_CLASS(rewd_controllers::MoveUntilTouchCartVelocityController,
                       controller_interface::ControllerBase)
