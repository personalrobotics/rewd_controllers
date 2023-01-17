cmake_minimum_required(VERSION 2.8.3)
project(rewd_controllers)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")

#==============================================================================
# Dependencies
#
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  control_toolbox
  controller_interface
  hardware_interface
  pr_control_msgs
  pr_hardware_interfaces
  realtime_tools
  eigen_conversions
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

if ("${controller_interface_VERSION}" VERSION_LESS 0.10.0)
  message(FATAL_ERROR "controller_interface package must be at least version 0.10.0, not packaged in Indigo. Please upgrade to Jade or Kinetic or build the ros_control packages from source.")
endif()

find_package(aikido REQUIRED COMPONENTS common control control_ros io statespace trajectory)
include_directories(SYSTEM ${aikido_INCLUDE_DIRS})

find_package(DART 6.6.2 REQUIRED COMPONENTS utils utils-urdf optimizer-nlopt CONFIG)
include_directories(SYSTEM ${DART_INCLUDE_DIRS})

set(LUCA_PATH "/home/rkjenamani/controller_ws/src/compliant-controller/thirdParty/lucadynamics")

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${LUCA_PATH}/include
  ${LUCA_PATH}/include/tinyxml
  ${LUCA_PATH}/include/luca_dynamics
)

#for luca dynamics
find_library(TINYXML
  NAMES libtinyxml.so
  PATHS ${LUCA_PATH}/lib
  REQUIRED
)

find_library(LUCA_DYN
  NAMES libluca_dyn.so
  PATHS ${LUCA_PATH}/lib
  # REQUIRED COMPONENTS tinyxml
)

link_libraries(
  # ${catkin_LIBRARIES}
  Threads::Threads
  ${LUCA_DYN}
  ${TINYXML} 
  )


# find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# find_library(
#         LUCADYN
#         NAMES libluca_dyn.a
#         HINTS thirdParty/lucadynamics/lib
#         REQUIRED
# )

# find_library(
#         TINYXML
#         NAMES  libtinyxml.a
#         HINTS thirdParty/lucadynamics/lib
#         REQUIRED
# )
# link_libraries(Eigen3::Eigen ${LUCADYN} ${TINYXML})
#==============================================================================
# Catkin Configuration
#
catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    pr_control_msgs
    pr_hardware_interfaces
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
  src/JointGroupPositionController.cpp
  src/GravityCompensationController.cpp
  src/helpers.cpp
  src/JointAdapter.cpp
  src/JointAdapterFactory.cpp
  src/JointGroupCommandControllerBase.cpp
  src/JointGroupCommandController.cpp
  src/JointTrajectoryControllerBase.cpp
  src/JointTrajectoryController.cpp
  src/MoveUntilTouchController.cpp
  src/MoveUntilTouchTopicJointTrajectoryController.cpp
  src/MoveUntilTouchTopicJointGroupCommandController.cpp
  src/FTThresholdClient.cpp
  src/FTThresholdServer.cpp
  src/TaskSpaceCompliantController.cpp
)
target_link_libraries("${PROJECT_NAME}"
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
)

#==============================================================================
# Format
#

# list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")
# include(ClangFormat)

# clang_format_setup(VERSION 6.0)

# if (CLANG_FORMAT_EXECUTABLE)
#   file(GLOB_RECURSE ALL_SOURCE_FILES
#     LIST_DIRECTORIES false
#     include/*.h include/*.hpp src/*.c src/*.cpp tests/*.cpp tests/*.hpp)

#   clang_format_add_sources(${ALL_SOURCE_FILES})

#   clang_format_add_targets()
# endif()

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
