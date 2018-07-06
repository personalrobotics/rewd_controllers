#ifndef REWD_CONTROLLERS_TRIGGERSTATE_HPP_
#define REWD_CONTROLLERS_TRIGGERSTATE_HPP_

namespace rewd_controllers {

/// \brief State of the Trigger request
///
/// TRIGGER_IDLE: no request active
/// TRIGGER_REQUESTED: trigger request received but not currently processing
/// TRIGGER_PENDING: trigger request underway
enum TriggerState { TRIGGER_IDLE, TRIGGER_REQUESTED, TRIGGER_PENDING };

}

#endif // REWD_CONTROLLERS_TRIGGERSTATE_HPP_
