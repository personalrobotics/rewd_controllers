#ifndef REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLER_HPP_
#define REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLER_HPP_

#include <rewd_controllers/MultiInterfaceController.hpp>
#include <rewd_controllers/JointTrajectoryControllerBase.hpp>

namespace rewd_controllers
{
class JointTrajectoryController
    : public MultiInterfaceController<hardware_interface::
                                          PositionJointInterface,
                                      hardware_interface::
                                          VelocityJointInterface,
                                      hardware_interface::EffortJointInterface,
                                      hardware_interface::JointStateInterface>,
      public JointTrajectoryControllerBase
{
public:
  JointTrajectoryController();
  virtual ~JointTrajectoryController();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period) override;
};

}  // namespace rewd_controllers

#endif  // REWD_CONTROLLERS_JOINTTRAJECTORYCONTROLLER_HPP_
