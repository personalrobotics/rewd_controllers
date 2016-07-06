#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <rewd_controllers/helpers.hpp>

namespace rewd_controllers {

//=============================================================================
dart::dynamics::SkeletonPtr loadRobotFromParameter(
  ros::NodeHandle& nodeHandle, const std::string& nameParameter)
{
  using aikido::util::CatkinResourceRetriever;

  static const dart::common::Uri emptyUri{};

  // Get the name of the "robot_description" parameter.
  std::string parameterName;
  nodeHandle.param<std::string>(
    nameParameter, parameterName, "/robot_description");

  // Load the URDF from the parameter server.
  std::string robotDescription;
  if (!nodeHandle.getParam(parameterName, robotDescription))
  {
    ROS_ERROR_STREAM("Failed loading URDF from '" << parameterName
      << "' parameter.");
    return nullptr;
  }

  // Load the URDF as a DART model.
  dart::utils::DartLoader urdfLoader;
  const auto resourceRetriever = std::make_shared<CatkinResourceRetriever>();
  const auto skeleton = urdfLoader.parseSkeletonString(
    robotDescription, emptyUri, resourceRetriever);

  if (!skeleton)
  {
    ROS_ERROR_STREAM("Failed parsing URDF into a Skeleton.");
    return nullptr;
  }

  return skeleton;
}

//=============================================================================
std::vector<JointParameter> loadJointsFromParameter(
  ros::NodeHandle& nodeHandle,
  const std::string& jointsParameter,
  const std::string& defaultType)
{
  using XmlRpc::XmlRpcValue;

  static const std::vector<JointParameter> emptyResult;

  XmlRpcValue jointsXml;

  if (jointsXml.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace()
      << "/joints' is not an array.");
    return emptyResult;
  }

  std::vector<JointParameter> output;
  for (int i = 0; i < jointsXml.size(); ++i)
  {
    JointParameter jointParameters;
    auto& jointXml = jointsXml[i];

    // Simple case where everything is effort-controlled.
    if (jointXml.getType() == XmlRpcValue::TypeString)
    {
      jointParameters.mName = static_cast<std::string>(jointXml);
      jointParameters.mType = defaultType;
    }
    // Advanced case where there are heterogeneous actuator types.
    else if (jointXml.getType() == XmlRpcValue::TypeStruct)
    {
      auto& nameXml = jointXml["name"];
      if (nameXml.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace() << "/joints["
          << i << "]/name' is not a string.");
        return emptyResult;
      }
      jointParameters.mName = static_cast<std::string>(nameXml);

      auto& typeXml = jointXml["type"];
      if (typeXml.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace() << "/joints["
          << i << "]/type' is not a string.");
        return emptyResult;
      }
      jointParameters.mType = static_cast<std::string>(typeXml);
    }
    else
    {
      ROS_ERROR_STREAM("Parameter '" << nodeHandle.getNamespace() << "/joints["
        << i << "]' is not a struct.");
      return emptyResult;
    }

    output.emplace_back(jointParameters);
  }

  return output;
}

//=============================================================================
dart::dynamics::MetaSkeletonPtr getControlledMetaSkeleton(
  const dart::dynamics::SkeletonPtr& skeleton,
  const std::vector<JointParameter>& parameters,
  const std::string& name)
{
  using dart::dynamics::Group;

  std::vector<dart::dynamics::DegreeOfFreedom*> dofs;
  dofs.reserve(parameters.size());

  for (const auto& param : parameters)
  {
    const auto& dofName = param.mName;
    const auto dof = skeleton->getDof(dofName);
    if (!dof)
    {
      ROS_ERROR_STREAM(
        "Skeleton has no DegreeOfFreedom named '" << dofName << "'.");
      return nullptr;
    }

    dofs.emplace_back(dof);
  }

  const auto controlledMetaSkeleton = Group::create(name, dofs, false, false);
  if (!controlledMetaSkeleton)
  {
    ROS_ERROR_STREAM("Failed creating MetaSkeleton of controlled DOFs.");
    return nullptr;
  }

  return controlledMetaSkeleton;
}

//=============================================================================
SkeletonJointStateUpdater::SkeletonJointStateUpdater(
      dart::dynamics::SkeletonPtr skeleton,
      hardware_interface::JointStateInterface* jointStateInterface)
  : mSkeleton{skeleton}
  , mDefaultPosition(skeleton->getNumDofs())
  , mDefaultVelocity(skeleton->getNumDofs())
  , mDefaultEffort(skeleton->getNumDofs())
{
  std::set<std::string> missingJointNames;

  mHandles.reserve(skeleton->getNumDofs());

  for (size_t idof = 0; idof < skeleton->getNumDofs(); ++idof)
  {
    const auto dof = skeleton->getDof(idof);
    const auto dofName = dof->getName();

    mDefaultPosition[idof] = dof->getInitialPosition();
    mDefaultVelocity[idof] = dof->getInitialVelocity();
    mDefaultEffort[idof] = 0.;

    hardware_interface::JointStateHandle handle;
    try
    {
      handle = jointStateInterface->getHandle(dofName);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      missingJointNames.emplace(dofName);

      // Use the default position, velocity, and effort.
      handle = hardware_interface::JointStateHandle{
        dofName, &mDefaultPosition[idof], &mDefaultVelocity[idof],
        &mDefaultEffort[idof]};
    }

    mHandles.emplace_back(handle);
  }

  if (!mHandles.empty())
  {
    std::stringstream msg;
    msg << "Failed to get JointStateHandles for " << missingJointNames.size()
        << " joints. The following joints will be assumed to have their"
        << " position and velocity for dynamics calculations:";

    for (const auto& dofName : missingJointNames)
      msg << " '" << dofName << "'";

    ROS_WARN_STREAM(msg.str());
  }
}

//=============================================================================
void SkeletonJointStateUpdater::update()
{
  for (size_t idof = 0; idof < mSkeleton->getNumDofs(); ++idof)
  {
    const auto dof = mSkeleton->getDof(idof);
    const auto& jointStateHandle = mHandles[idof];
    dof->setPosition(jointStateHandle.getPosition());
    dof->setVelocity(jointStateHandle.getVelocity());
    dof->setForce(jointStateHandle.getEffort());
  }
}

} // namespace rewd_controllers
