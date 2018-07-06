#ifndef REWD_CONTROLLERS_TRIGGERABLE_INTERFACE_HPP_
#define REWD_CONTROLLERS_TRIGGERABLE_INTERFACE_HPP_

#include <string>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <pr_hardware_interfaces/TriggerState.h>

namespace rewd_controllers {

class TriggerableHandle
{
public:
  TriggerableHandle();
  TriggerableHandle(const std::string& name, TriggerState* trigger_state);

  std::string getName() const;
  void trigger();
  bool isTriggerComplete();

private:
  std::string name_;
  TriggerState* trigger_state_;
};

class TriggerableInterface :
    public hardware_interface::HardwareResourceManager<TriggerableHandle, hardware_interface::ClaimResources>{};

}

#endif // REWD_CONTROLLERS_TRIGGERABLE_INTERFACE_HPP_
