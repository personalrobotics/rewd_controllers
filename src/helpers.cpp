#include <aikido/io/CatkinResourceRetriever.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <rewd_controllers/helpers.hpp>

namespace rewd_controllers {

//=============================================================================
dart::dynamics::SkeletonPtr
loadRobotFromParameter(const ros::NodeHandle &nodeHandle,
                       const std::string &nameParameter) {
  using aikido::io::CatkinResourceRetriever;

  static const dart::common::Uri emptyUri{};

  // Get the name of the "robot_description" parameter.
  std::string parameterName;
  nodeHandle.param<std::string>(nameParameter, parameterName,
                                "/robot_description");

  // Load the URDF from the parameter server.
  std::string robotDescription;
  if (!nodeHandle.getParam(parameterName, robotDescription)) {
    ROS_ERROR_STREAM("Failed loading URDF from '" << parameterName
                                                  << "' parameter.");
    return nullptr;
  }

  // Load the URDF as a DART model.
  dart::utils::DartLoader urdfLoader;
  const auto resourceRetriever = std::make_shared<CatkinResourceRetriever>();
  const auto skeleton = urdfLoader.parseSkeletonString(
      robotDescription, emptyUri, resourceRetriever);

  if (!skeleton) {
    ROS_ERROR_STREAM("Failed parsing URDF into a Skeleton.");
    return nullptr;
  }

  return skeleton;
}

//=============================================================================
std::vector<JointParameter>
loadJointsFromParameter(const ros::NodeHandle &nodeHandle,
                        const std::string &jointsParameter,
                        const std::string &defaultType) {
  using XmlRpc::XmlRpcValue;

  static const std::vector<JointParameter> emptyResult;

  XmlRpcValue jointsXml;
  if (!nodeHandle.getParam("joints", jointsXml)) {
    ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                   << "/joints' is required.");
  }

  if (jointsXml.getType() != XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                   << "/joints' is not an array.");
    return emptyResult;
  }

  std::vector<JointParameter> output;
  for (int i = 0; i < jointsXml.size(); ++i) {
    JointParameter jointParameters;
    auto &jointXml = jointsXml[i];

    // Simple case where everything is effort-controlled.
    if (jointXml.getType() == XmlRpcValue::TypeString) {
      jointParameters.mName = static_cast<std::string>(jointXml);
      jointParameters.mType = defaultType;
    }
    // Advanced case where there are heterogeneous actuator types.
    else if (jointXml.getType() == XmlRpcValue::TypeStruct) {
      auto &nameXml = jointXml["name"];
      if (nameXml.getType() != XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                       << "/joints[" << i
                                       << "]/name' is not a string.");
        return emptyResult;
      }
      jointParameters.mName = static_cast<std::string>(nameXml);

      auto &typeXml = jointXml["type"];
      if (typeXml.getType() != XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
                                       << "/joints[" << i
                                       << "]/type' is not a string.");
        return emptyResult;
      }
      jointParameters.mType = static_cast<std::string>(typeXml);
    } else {
      ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace() << "/joints["
                                     << i << "]' is not a struct.");
      return emptyResult;
    }

    output.emplace_back(jointParameters);
  }

  return output;
}

//=============================================================================
std::unordered_map<std::string, double> loadGoalConstraintsFromParameter(
    const ros::NodeHandle &nodeHandle,
    const std::vector<JointParameter> &jointParameters) {
  std::unordered_map<std::string, double> goalConstraints;

  for (const auto &jointParam : jointParameters) {
    std::string jointName = jointParam.mName;

    double goalConstraint;
    if (nodeHandle.getParam("constraints/" + jointName + "/goal",
                            goalConstraint)) {
      goalConstraints[jointName] = goalConstraint;
    }
  }
  if (goalConstraints.empty()) {
    ROS_WARN("No goal tolerance constraints specified.");
  } else {
    ROS_INFO_STREAM("Goal tolerance constraints loaded for "
                    << goalConstraints.size() << " joints.");
  }
  return goalConstraints;
}

//=============================================================================
std::unordered_map<std::string, double> loadTrajectoryConstraintsFromParameter(
    const ros::NodeHandle &nodeHandle,
    const std::vector<JointParameter> &jointParameters) {
  std::unordered_map<std::string, double> trajConstraints;

  for (const auto &jointParam : jointParameters) {
    std::string jointName = jointParam.mName;

    double trajConstraint;
    if (nodeHandle.getParam("constraints/" + jointName + "/trajectory",
                            trajConstraint)) {
      trajConstraints[jointName] = trajConstraint;
    }
  }

  if (trajConstraints.empty()) {
    ROS_WARN("No trajectory tolerance constraints specified.");
  } else {
    ROS_INFO_STREAM("Trajectory tolerance constraints loaded for "
                    << trajConstraints.size() << " joints.");
  }
  return trajConstraints;
}

//=============================================================================
dart::dynamics::MetaSkeletonPtr
getControlledMetaSkeleton(const dart::dynamics::SkeletonPtr &skeleton,
                          const std::vector<JointParameter> &parameters,
                          const std::string &name) {
  using dart::dynamics::Group;

  std::vector<dart::dynamics::DegreeOfFreedom *> dofs;
  dofs.reserve(parameters.size());

  for (const auto &param : parameters) {
    const auto &dofName = param.mName;
    const auto dof = skeleton->getDof(dofName);
    if (!dof) {
      ROS_ERROR_STREAM("Skeleton has no DegreeOfFreedom named '" << dofName
                                                                 << "'.");
      return nullptr;
    }

    dofs.emplace_back(dof);
  }

  const auto controlledMetaSkeleton = Group::create(name, dofs, false, true);
  if (!controlledMetaSkeleton) {
    ROS_ERROR_STREAM("Failed creating MetaSkeleton of controlled DOFs.");
    return nullptr;
  }

  if (controlledMetaSkeleton->getNumDofs() != parameters.size()) {
    ROS_ERROR_STREAM("Only single-DOF joints are supported.");
    return nullptr;
  }

  return controlledMetaSkeleton;
}

//=============================================================================
SkeletonJointStateUpdater::SkeletonJointStateUpdater(
    dart::dynamics::SkeletonPtr skeleton,
    hardware_interface::JointStateInterface *jointStateInterface)
    : mSkeleton{skeleton}, mDefaultPosition(skeleton->getNumDofs()),
      mDefaultVelocity(skeleton->getNumDofs()),
      mDefaultEffort(skeleton->getNumDofs()) {
  std::set<std::string> missingJointNames;

  mHandles.reserve(skeleton->getNumDofs());

  for (size_t idof = 0; idof < skeleton->getNumDofs(); ++idof) {
    const auto dof = skeleton->getDof(idof);
    const auto dofName = dof->getName();

    mDefaultPosition[idof] = dof->getInitialPosition();
    mDefaultVelocity[idof] = dof->getInitialVelocity();
    mDefaultEffort[idof] = 0.;

    hardware_interface::JointStateHandle handle;
    try {
      handle = jointStateInterface->getHandle(dofName);
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      missingJointNames.emplace(dofName);

      // Use the default position, velocity, and effort.
      handle = hardware_interface::JointStateHandle{
          dofName, &mDefaultPosition[idof], &mDefaultVelocity[idof],
          &mDefaultEffort[idof]};
    }

    mHandles.emplace_back(handle);
  }

  if (!mHandles.empty()) {
    std::stringstream msg;
    msg << "Failed to get JointStateHandles for " << missingJointNames.size()
        << " joints. The following joints will be assumed to have their"
        << " position and velocity for dynamics calculations:";

    for (const auto &dofName : missingJointNames)
      msg << " '" << dofName << "'";

    ROS_WARN_STREAM(msg.str());
  }
}

//=============================================================================
void SkeletonJointStateUpdater::update() {
  for (size_t idof = 0; idof < mSkeleton->getNumDofs(); ++idof) {
    const auto dof = mSkeleton->getDof(idof);
    const auto &jointStateHandle = mHandles[idof];
    dof->setPosition(jointStateHandle.getPosition());
    dof->setVelocity(jointStateHandle.getVelocity());
    dof->setForce(jointStateHandle.getEffort());
  }
}

ros::NodeHandle
createDefaultAdapterNodeHandle(const ros::NodeHandle &parentNodeHandle,
                               const dart::dynamics::DegreeOfFreedom *dof) {
  auto dofName = dof->getName();
  while (dofName.at(0) == '/') {
    // a leading slash creates a root level namespace
    // ignoring the parent workspace, which is
    // obviously not where we want to read parameters from
    dofName.erase(0, 1);
  }

  const ros::NodeHandle gainsNodeHandle{parentNodeHandle, "gains"};
  return ros::NodeHandle{gainsNodeHandle, dofName};
}

ExtendedJointPosition::ExtendedJointPosition(unsigned int numberOfInput_args, double threshold_of_change_args) {
  numberOfInput = numberOfInput_args;
  threshold_of_change = threshold_of_change_args;
  init_q.resize(numberOfInput, 1);
  extended_q.resize(numberOfInput, 1);
  previous_sensor_q.resize(numberOfInput, 1);
}

void ExtendedJointPosition::initializeExtendedJointPosition(const Eigen::MatrixXd& init_q_args)
{
  if (is_initialized == false)
  {
    init_q = normalizeJointPosition(init_q_args);
    extended_q = init_q;
    previous_sensor_q = init_q;
    is_initialized = true;
  }
}

void ExtendedJointPosition::initializeExtendedJointPosition(const double init_q_args, int dof)
{
  // if (is_initialized == false)
  // {
    init_q(0) = normalizeJointPosition(init_q_args);
    extended_q(0) = init_q(0);
    previous_sensor_q(0) = init_q(0);
    is_initialized = true;
  // }
}

double ExtendedJointPosition::normalizeJointPosition(double input)
{
  /*
  * Move the joint position inside the [-pi,pi).
  * args : double -> 1d case
  * args : VectorXd -> multiple dimension(=dof) case
  */
  // double output = input;
  while(input > M_PI)
  {
      input -= 2*M_PI;
  }
  while(input <= -M_PI)
  {
      input += 2*M_PI;
  }
  return input;
}

Eigen::VectorXd ExtendedJointPosition::normalizeJointPosition(const Eigen::VectorXd& input)
{
  Eigen::VectorXd output = input;
  for (int i = 0; i < numberOfInput; ++i) {
      output[i] = normalizeJointPosition(input[i]);
  }
  return output;
}

void ExtendedJointPosition::estimateExtendedJoint(const Eigen::VectorXd& current_sensor_q)
{
  for (int i = 0; i < numberOfInput; ++i) {
    if (abs(current_sensor_q[i] - previous_sensor_q(i)) >= threshold_of_change)
    {
      extended_q(i) += normalizeJointPosition(current_sensor_q[i]) - normalizeJointPosition(previous_sensor_q(i));
    }
    else
    {
      int howMuchRotate;
      if (extended_q(i) >= 0)
      {
        howMuchRotate = static_cast<int>(extended_q(i) / (2*M_PI));
      }
      else
      {
        howMuchRotate = static_cast<int>(extended_q(i) / (2*M_PI)) - 1;
      }
      extended_q(i) = (double)howMuchRotate * (2*M_PI) + current_sensor_q[i];
    }
  }
  previous_sensor_q = current_sensor_q;
}

void ExtendedJointPosition::estimateExtendedJoint(const double current_sensor_q, int dof)
{
  // for (int i = 0; i < numberOfInput; ++i) {
  // std::cout << "Extended_q: " << extended_q << " Current_sensor_q: " << current_sensor_q << std::endl;
  if (abs(current_sensor_q - previous_sensor_q(0)) >= threshold_of_change)
  {
    // std::cout << "Threshold exceedeed" << std::endl;
    extended_q(0) += normalizeJointPosition(current_sensor_q) - normalizeJointPosition(previous_sensor_q(0));
  }
  else
  {
    int howMuchRotate;
    if (extended_q(0) >= 0)
    {
      howMuchRotate = static_cast<int>(extended_q(0) / (2*M_PI));
    }
    else
    {
      howMuchRotate = static_cast<int>(extended_q(0) / (2*M_PI)) - 1;
    }
    // std::cout << "howMuchRotate: " << howMuchRotate << std::endl;
    extended_q(0) = (double)howMuchRotate * (2*M_PI) + current_sensor_q;
  }
  // }
  previous_sensor_q(0) = current_sensor_q;
}

Eigen::VectorXd ExtendedJointPosition::getExtendedJoint()
{
  return extended_q;
}

double ExtendedJointPosition::getExtendedJoint(int dof)
{
  return extended_q(0);
}

} // namespace rewd_controllers
