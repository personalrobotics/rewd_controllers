#ifndef REWD_CONTROLLERS_JOINTADAPTERFACTORY_HPP_
#define REWD_CONTROLLERS_JOINTADAPTERFACTORY_HPP_
#include <memory>
#include <string>
#include <unordered_map>
#include <dart/dynamics/dynamics.hpp>
#include <hardware_interface/robot_hw.h>
#include "JointAdapter.hpp"

namespace rewd_controllers {

class JointAdapterFactory final
{
public:
  JointAdapterFactory();
  ~JointAdapterFactory();

  template <class Interface, class Adapter>
  void registerFactory(const std::string& type);

  std::unique_ptr<JointAdapter> create(
      const std::string& type,
      hardware_interface::RobotHW* hardwareInterface,
      dart::dynamics::DegreeOfFreedom* dof) const;

private:
  using FactoryFunction = std::function<JointAdapter*(
      hardware_interface::RobotHW*, dart::dynamics::DegreeOfFreedom*)>;

  hardware_interface::RobotHW* mHardwareInterface;
  std::unordered_map<std::string, FactoryFunction> mFactories;
};

} // namespace rewd_controllers

#include "detail/JointAdapterFactory-impl.hpp"

#endif // ifndef REWD_CONTROLLERS_JOINTADAPTERFACTORY_HPP_
