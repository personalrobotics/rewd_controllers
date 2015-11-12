cmake_minimum_required(VERSION 2.8.3)
project(rewd_controllers)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  realtime_tools
  urdf
)

# TODO: filesystem shouldn't be required here.
find_package(Boost REQUIRED COMPONENTS filesystem thread)
find_package(DART REQUIRED COMPONENTS core)
find_package(r3 REQUIRED)

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${r3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    realtime_tools
    urdf
  DEPENDS
    r3
    DART
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
  src/joint_group_position_controller.cpp
  src/gravity_compensation_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${r3_LIBRARIES}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rewd_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
