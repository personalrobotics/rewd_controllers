#ifndef REWD_CONTROLLERS_HELPERS_HPP_
#define REWD_CONTROLLERS_HELPERS_HPP_
#include <typeinfo>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace rewd_controllers {

//=============================================================================
struct JointParameter
{
  std::string mName;
  std::string mType;
};

//=============================================================================
class JointAdapter
{
public:
  virtual ~JointAdapter() = default;
};

//=============================================================================
class JointPositionAdapter : public JointAdapter
{
public:
  explicit JointPositionAdapter(
    hardware_interface::PositionJointInterface* interface)
  {
  }
};

//=============================================================================
class JointEffortAdapter : public JointAdapter
{
public:
  explicit JointEffortAdapter(
    hardware_interface::EffortJointInterface* interface)
  {
  }
};

//=============================================================================
class JointAdapterFactory final
{
public:
  JointAdapterFactory() = default;
  ~JointAdapterFactory() = default;

  template <class Interface, class Adapter>
  void registerFactory(const std::string& type)
  {
    mFactories.emplace(
      type, [](hardware_interface::RobotHW* hardwareInterface)
      {
        const auto interface = hardwareInterface->get<Interface>();
        if (!interface)
        {
          ROS_ERROR_STREAM("RobotHW has no interface of type '"
            << typeid(Interface).name() << "'.");
          return nullptr;
        }

        return new Adapter{interface};
      }
    );
  }

  std::unique_ptr<JointAdapter> create(const std::string& type,
    hardware_interface::RobotHW* hardwareInterface) const
  {
    const auto it = mFactories.find(type);
    if (it == std::end(mFactories))
    {
      ROS_ERROR_STREAM("Unknown joint type '" << type << "'.");
      return nullptr;
    }
    
    const auto& factoryFunction = it->second;
    return std::unique_ptr<JointAdapter>{factoryFunction(hardwareInterface)};
  }

private:
  using FactoryFunction = std::function<
    JointAdapter* (hardware_interface::RobotHW*)>;
  
  hardware_interface::RobotHW* mHardwareInterface;
  std::unordered_map<std::string, FactoryFunction> mFactories;
};

class SkeletonJointStateUpdater final
{
public:
  SkeletonJointStateUpdater(
    dart::dynamics::SkeletonPtr skeleton,
    hardware_interface::JointStateInterface* jointStateInterface);

  void update();

private:
  dart::dynamics::SkeletonPtr mSkeleton;
  Eigen::VectorXd mDefaultPosition;
  Eigen::VectorXd mDefaultVelocity;
  Eigen::VectorXd mDefaultEffort;
  std::vector<hardware_interface::JointStateHandle> mHandles;
};

//=============================================================================
dart::dynamics::SkeletonPtr loadRobotFromParameter(
  ros::NodeHandle& nodeHandle, const std::string& nameParameter);

std::vector<JointParameter> loadJointsFromParameter(
  ros::NodeHandle& nodeHandle, const std::string& jointsParameter);

dart::dynamics::MetaSkeletonPtr getControlledMetaSkeleton(
  const dart::dynamics::SkeletonPtr& skeleton,
  const std::vector<JointParameter>& parameters,
  const std::string& name,
  const std::string& defaultType);

} // namespace rewd_controllers

#endif // ifndef REWD_CONTROLLERS_HELPERS_HPP_
