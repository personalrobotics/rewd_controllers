#include <rewd_controllers/TriggerableInterface.hpp>

namespace rewd_controllers {

TriggerableHandle::TriggerableHandle()
  : trigger_state_(NULL)
{}

TriggerableHandle::TriggerableHandle(const std::string& name, TriggerState* trigger_state)
  : name_(name)
  , trigger_state_(trigger_state)
{
  if (!trigger_state_) {
    throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                         "'. Trigger state pointer is null.");
  }
}

std::string TriggerableHandle::getName() const {return name_;}

void TriggerableHandle::trigger()
{
  if (isTriggerComplete()) { // ignore double triggers
    *trigger_state_ = TRIGGER_REQUESTED;
  }
}

bool TriggerableHandle::isTriggerComplete()
{
  assert(trigger_state_);
  return *trigger_state_ == TRIGGER_IDLE;
}

} // namespace rewd_controllers
