namespace rewd_controllers {

//=============================================================================
template <class Interface, class Adapter>
void JointAdapterFactory::registerFactory(const std::string& type)
{
  mFactories.emplace(
      type,
      [](hardware_interface::RobotHW* hardwareInterface,
         dart::dynamics::DegreeOfFreedom* dof) -> JointAdapter* {
        const auto interface = hardwareInterface->get<Interface>();
        if (!interface)
        {
          ROS_ERROR_STREAM(
              "RobotHW has no interface of type '" << typeid(Interface).name()
                                                   << "'.");
          return nullptr;
        }

        const auto dofName = dof->getName();
        typename Interface::ResourceHandleType handle;
        try
        {
          handle = interface->getHandle(dofName);
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
          ROS_ERROR_STREAM(
              "Unable to get interface of type '" << typeid(Interface).name()
                                                  << "' for joint '"
                                                  << dofName
                                                  << "'.");
          return nullptr;
        }

        return new Adapter{handle, dof};
      });
}

} // namespace rewd_controllers
