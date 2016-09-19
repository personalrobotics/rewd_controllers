///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/**
 * \author Adolfo Rodríguez Tsouroukdissian
 * \author Clint Liddick
 */

#ifndef REWD_CONTROLLERS_MULTI_INTERFACE_CONTROLLER_H
#define REWD_CONTROLLERS_MULTI_INTERFACE_CONTROLLER_H

#include <algorithm>
#include <sstream>
#include <typeinfo>

#include <controller_interface/controller_base.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>

namespace rewd_controllers
{
using controller_interface::ControllerBase;
/** \cond HIDDEN_SYMBOLS */
namespace internal
{
template <class T>
std::string enumerateElements(const T& val, const std::string& delimiter = ", ",
                              const std::string& prefix = "",
                              const std::string& suffix = "");

}  // namespace
   /** \endcond */

/**
 * \brief %Controller able to claim resources from multiple hardware interfaces.
 *
 * This particular controller implementation allows to claim resources from one
 * up to four different hardware interfaces. The types of these hardware
 * interfaces are specified as template parameters.
 *
 * An example multi-interface controller could claim, for instance, resources
 * from a position-controlled arm and velocity-controlled wheels. Another
 * example would be a controller claiming both position and effort interfaces
 * for the same robot resources, but this would require a robot with a custom
 * (non-exclusive) resource handling policy.
 *
 * By default, all specified hardware interfaces are required, and their
 * existence will be enforced by \ref initRequest. It is possible to make
 *hardware
 * interfaces optional by means of the \c allow_optional_interfaces
 * \ref MultiInterfaceController::MultiInterfaceController "constructor"
 *parameter.
 * This allows to write controllers where some interfaces are mandatory, and
 * others, if present, improve controller performance, but whose absence does
 *not
 * prevent the controller from running.
 *
 * The following is an example of a controller claiming resources from velocity-
 * and effort-controlled joints.
 *
 * \code
 * #include <controller_interface/multi_interface_controller.h>
 * #include <hardware_interface/joint_command_interface.h>
 *
 * using namespace hardware_interface;
 * class VelEffController : public
 *       controller_interface::MultiInterfaceController<VelocityJointInterface,
 *                                                      EffortJointInterface>
 * {
 * public:
 *   VelEffController() {}
 *
 *   bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
 *   {
 *     // robot_hw pointer only contains the two interfaces requested by the
 *     // controller. It is a subset of the entire robot, which may have more
 *     // hardware interfaces
 *
 *     // v and e below are guarranteed to be valid
 *     VelocityJointInterface* v = robot_hw->get<VelocityJointInterface>();
 *     EffortJointInterface*   e = robot_hw->get<EffortJointInterface>();
 *
 *     // Fetch resources from interfaces, perform rest of initialization
 *     //...
 *
 *     return true;
 *   }
 *   void starting(const ros::Time& time);
 *   void update(const ros::Time& time, const ros::Duration& period);
 *   void stopping(const ros::Time& time);
 * };
 * \endcode
 *
 * The following fragment is a modified version of the above example, where
 * controller interfaces are not required. It is left to the controller
 * implementer to verify interface validity. Only the initialization code is
 * shown.
 *
 * \code
 * class VelEffController : public
 *       controller_interface::MultiInterfaceController<VelocityJointInterface,
 *                                                      EffortJointInterface>
 * {
 * public:
 *   // Note true flag passed to parent class, allowing requested hardware
 *   // interfaces to be optional
 *   VelEffController()
 *    : controller_interface::MultiInterfaceController<VelocityJointInterface,
 *                                                     EffortJointInterface>
 *(true)
 *   {}
 *
 *   bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
 *   {
 *     // robot_hw pointer contains at most the two interfaces requested by the
 *     // controller. It may have none, only one or both, depending on whether
 *the
 *     // robot exposes them
 *
 *     // v is a required interface
 *     VelocityJointInterface* v = robot_hw->get<VelocityJointInterface>();
 *     if (!v)
 *     {
 *       return false;
 *     }
 *
 *     // e is an optional interface. If present, additional features are
 *enabled.
 *     // Controller can still function if interface or some of its resources
 *are
 *     // absent
 *     EffortJointInterface* e = robot_hw->get<EffortJointInterface>();
 *
 *     // Fetch resources from interfaces, perform rest of initialization
 *     //...
 *
 *     return true;
 *   }
 *   ...
 * };
 * \endcode
 *
 * \tparam T1 Hardware interface type.
 * This parameter is \e required.
 *
 * \tparam Ts Additional hardware interface types.
 * These parameters are \e optional. Leave unspecified if controller only claims
 * resources from a \e single hardware interface.
 *
 * \pre When specified, template parameters \c T1 and all \c Ts must be different
 * types.
 */
template <class T1, class... Ts>
class MultiInterfaceController : public ControllerBase
{
public:
  /**
   * \param allow_optional_interfaces If set to true, \ref initRequest will
   * not fail if one or more of the requested interfaces is not present.
   * If set to false (the default), all requested interfaces are required.
   */
  MultiInterfaceController(bool allow_optional_interfaces = false)
      : allow_optional_interfaces_(allow_optional_interfaces)
  {
    state_ = CONSTRUCTED;
  }

  virtual ~MultiInterfaceController() {}

  /** \name Non Real-Time Safe Functions
   *\{*/

  /**
   * \brief Custom controller initialization logic.
   *
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   *
   * \param robot_hw Robot hardware abstraction containing a subset of the
   *entire
   * robot. If \ref MultiInterfaceController::MultiInterfaceController
   * "MultiInterfaceController" was called with \c allow_optional_interfaces set
   * to \c false (the default), this parameter contains all the interfaces
   * requested by the controller.
   * If \c allow_optional_interfaces was set to \c false, this parameter may
   * contain none, some or all interfaces requested by the controller, depending
   * on whether the robot exposes them. Please refer to the code examples in the
   * \ref MultiInterfaceController "class description".
   *
   * \param controller_nh A NodeHandle in the namespace from which the
   *controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(hardware_interface::RobotHW* /*robot_hw*/,
                    ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }

  /**
   * \brief Custom controller initialization logic.
   *
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   *
   * \param robot_hw Robot hardware abstraction containing a subset of the
   *entire
   * robot. If \ref MultiInterfaceController::MultiInterfaceController
   * "MultiInterfaceController" was called with \c allow_optional_interfaces set
   * to \c false (the default), this parameter contains all the interfaces
   * requested by the controller.
   * If \c allow_optional_interfaces was set to \c false, this parameter may
   * contain none, some or all interfaces requested by the controller, depending
   * on whether the robot exposes them. Please refer to the code examples in the
   * \ref MultiInterfaceController "class description".
   *
   * \param root_nh A NodeHandle in the root of the controller manager
   *namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers,
   *services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool init(hardware_interface::RobotHW* /*robot_hw*/,
                    ros::NodeHandle& /*root_nh*/,
                    ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }

protected:
  /**
   * \brief Initialize the controller from a RobotHW pointer.
   *
   * This calls \ref init with a RobotHW that is a subset of the input
   * \c robot_hw parameter, containing only the requested hardware interfaces
   * (all or some, depending on the value of \c allow_optional_interfaces passed
   * to the constructor).
   *
   * \param robot_hw The robot hardware abstraction.
   *
   * \param root_nh A NodeHandle in the root of the controller manager
   *namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers,
   *services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& root_nh,
                           ros::NodeHandle& controller_nh,
                           ClaimedResources& claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED) {
      ROS_ERROR(
          "Cannot initialize this controller because it failed to be "
          "constructed");
      return false;
    }

    // check for required hardware interfaces
    if (!allow_optional_interfaces_ && !hasRequiredInterfaces(robot_hw)) {
      return false;
    }

    // populate robot hardware abstraction containing only controller hardware
    // interfaces (subset of robot)
    hardware_interface::RobotHW* robot_hw_ctrl_p = &robot_hw_ctrl_;
    extractInterfaceResources(robot_hw, robot_hw_ctrl_p);

    // custom controller initialization
    clearClaims(
        robot_hw_ctrl_p);  // claims will be populated on controller init
    if (!init(robot_hw_ctrl_p, controller_nh)
        || !init(robot_hw_ctrl_p, root_nh, controller_nh)) {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }

    // populate claimed resources
    claimed_resources.clear();
    populateClaimedResources(robot_hw_ctrl_p, claimed_resources);
    clearClaims(robot_hw_ctrl_p);
    // NOTE: Above, claims are cleared since we only want to know what they are
    // and report them back
    // as an output parameter. Actual resource claiming by the controller is
    // done when the controller
    // is start()ed

    // initialization successful
    state_ = INITIALIZED;
    return true;
  }

  /*\}*/

  /**
   * \brief Check if robot hardware abstraction contains all required
   * interfaces.
   * \param robot_hw Robot hardware abstraction.
   * \return true if all required hardware interfaces are exposed by \c
   * robot_hw,
   * false otherwise.
   */
  static bool hasRequiredInterfaces(hardware_interface::RobotHW* robot_hw)
  {
    std::vector<hardware_interface::HardwareInterface*> hws{
        robot_hw->get<T1>(), (robot_hw->get<Ts>())...};
    bool hasAll = true;
    for (auto hw : hws) {
      if (!hw) {
        auto hw_type =
            hardware_interface::internal::demangleSymbol(typeid(*hw).name());
        ROS_ERROR_STREAM(
            "This controller requires a hardware interface of type '"
            << hw_type << "', "
            << "but is not exposed by the robot. Available interfaces in "
               "robot:\n"
            << internal::enumerateElements(robot_hw->getNames(), "\n", "- '",
                                           "'"));  // delimiter, prefix, suffux
        hasAll = false;
      }
    }
    return hasAll;
  }

  /**
   * \brief Clear claims from all hardware interfaces requested by this
   * controller.
   * \param robot_hw Robot hardware abstraction containing the interfaces whose
   * claims will be cleared.
   */
  static void clearClaims(hardware_interface::RobotHW* robot_hw)
  {
    std::vector<hardware_interface::HardwareInterface*> hws{
        robot_hw->get<T1>(), (robot_hw->get<Ts>())...};
    for (auto hw : hws) {
      if (hw) {
        hw->clearClaims();
      }
    }
  }

  /**
   * \brief Extract all hardware interfaces requested by this controller from
   *  \c robot_hw_in, and add them also to \c robot_hw_out.
   * \param[in] robot_hw_in Robot hardware abstraction containing the interfaces
   * requested by this controller, and potentially others.
   * \param[out] robot_hw_out Robot hardware abstraction containing \e only the
   * interfaces requested by this controller.
   */
  static void extractInterfaceResources(
      hardware_interface::RobotHW* robot_hw_in,
      hardware_interface::RobotHW* robot_hw_out)
  {
    std::vector<hardware_interface::HardwareInterface*> hws{
        robot_hw_in->get<T1>(), (robot_hw_in->get<Ts>())...};
    for (auto hw : hws) {
      if (hw) {
        robot_hw_out->registerInterface(hw);
      }
    }
  }

  /**
   * \brief Extract all hardware interfaces requested by this controller from
   *  \c robot_hw and claim them.
   * \param robot_hw Robot hardware abstraction containing the interfaces
   * requested by this controller, and potentially others.
   * \param claimed_resources The resources claimed by this controller.
   * They can belong to multiple hardware interfaces.
   */
  static void populateClaimedResources(hardware_interface::RobotHW* robot_hw,
                                       ClaimedResources& claimed_resources)
  {
    std::vector<hardware_interface::HardwareInterface*> hws{
        robot_hw->get<T1>(), (robot_hw->get<Ts>())...};
    for (auto hw : hws) {
      hardware_interface::InterfaceResources iface_res;
      iface_res.hardware_interface =
          hardware_interface::internal::demangleSymbol(typeid(*hw).name());
      iface_res.resources = hw->getClaims();
      claimed_resources.push_back(iface_res);
    }
  }

  /** Robot hardware abstraction containing only the subset of interfaces
   * requested by the controller. */
  hardware_interface::RobotHW robot_hw_ctrl_;

  /** Flag to indicate if hardware interfaces are considered optional (i.e.
   * non-required). */
  bool allow_optional_interfaces_;

private:
  MultiInterfaceController(const MultiInterfaceController& c);
  MultiInterfaceController& operator=(const MultiInterfaceController& c);
};

namespace internal
{
template <class T>
inline std::string enumerateElements(const T& val, const std::string& delimiter,
                                     const std::string& prefix,
                                     const std::string& suffix)
{
  std::string ret;
  if (val.empty()) {
    return ret;
  }

  const std::string sdp = suffix + delimiter + prefix;
  std::stringstream ss;
  ss << prefix;
  std::copy(val.begin(), val.end(),
            std::ostream_iterator<typename T::value_type>(ss, sdp.c_str()));
  ret = ss.str();
  if (!ret.empty()) {
    ret.erase(ret.size() - delimiter.size() - prefix.size());
  }
  return ret;
}

}  // namespace

}  // namespace

#endif
