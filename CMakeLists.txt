cmake_minimum_required(VERSION 2.8.3)
project(rewd_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  control_toolbox
  # effort_controllers
  realtime_tools
  urdf
  kdl_extension
  kdl_parser
)

find_package(Boost REQUIRED COMPONENTS
  thread
)
find_package(orocos_kdl REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    # effort_controllers
    realtime_tools
    urdf
    forward_command_controller
    kdl_extension
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/joint_group_position_controller.cpp
  src/${PROJECT_NAME}/RosMsgConverter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES rewd_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
