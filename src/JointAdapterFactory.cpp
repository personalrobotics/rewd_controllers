#include <rewd_controllers/JointAdapterFactory.hpp>

namespace rewd_controllers {

//=============================================================================
JointAdapterFactory::JointAdapterFactory() {}

//=============================================================================
JointAdapterFactory::~JointAdapterFactory() {}

//=============================================================================
std::unique_ptr<JointAdapter>
JointAdapterFactory::create(const std::string &type,
                            hardware_interface::RobotHW *hardwareInterface,
                            dart::dynamics::DegreeOfFreedom *dof) const {
  const auto it = mFactories.find(type);
  if (it == std::end(mFactories)) {
    ROS_ERROR_STREAM("Unknown joint type '" << type << "'.");
    return nullptr;
  }

  const auto &factoryFunction = it->second;
  return std::unique_ptr<JointAdapter>{factoryFunction(hardwareInterface, dof)};
}

} // namespace rewd_controllers
