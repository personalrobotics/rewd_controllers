cmake_minimum_required(VERSION 2.8.3)
project(rewd_controllers)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")

#==============================================================================
# Dependencies
#
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  control_toolbox
  controller_interface
  realtime_tools
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# TODO: Temporarily disabled.
#if ("${controller_interface_VERSION}" VERSION_LESS 0.10.0)
#  message(FATAL_ERROR "controller_interface package must be at least version 0.10.0, not packaged in Indigo. Please upgrade to Jade or build the ros_control packages from source.") 
#endif()

find_package(aikido REQUIRED COMPONENTS control_ros statespace trajectory util)
include_directories(SYSTEM ${aikido_INCLUDE_DIRS})

find_package(DART REQUIRED CONFIG)
include_directories(SYSTEM ${DART_INCLUDE_DIRS})

#==============================================================================
# Catkin Configuration
#
catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    control_toolbox
    controller_interface
    realtime_tools
  DEPENDS
    aikido
    DART
  INCLUDE_DIRS
    include
  LIBRARIES
    "${PROJECT_NAME}"
)

#==============================================================================
# Targets
#
add_library("${PROJECT_NAME}" SHARED
  # TODO: Temporarily disabled.
  #src/joint_group_position_controller.cpp
  #src/gravity_compensation_controller.cpp
  src/helpers.cpp
  src/JointAdapter.cpp
  src/JointAdapterFactory.cpp
  src/JointTrajectoryController.cpp
)
target_link_libraries("${PROJECT_NAME}"
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
)

#==============================================================================
# Installation
#
install(DIRECTORY "include"
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
install(TARGETS "${PROJECT_NAME}"
  ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
install(FILES "rewd_controllers_plugins.xml"
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)
