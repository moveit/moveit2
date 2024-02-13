cmake_minimum_required(VERSION 3.22)
project(pilz_industrial_motion_planner_testutils LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include)

# Declare a C++ library
add_library(
  pilz_industrial_motion_planner_testutils SHARED
  src/cartesianconfiguration.cpp src/jointconfiguration.cpp
  src/robotconfiguration.cpp src/sequence.cpp src/xml_testdata_loader.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(
  pilz_industrial_motion_planner_testutils
  Boost
  Eigen3
  eigen3_cmake_module
  rclcpp
  moveit_core
  moveit_msgs
  tf2_eigen)

# ##############################################################################
# Install ##
# ##############################################################################

install(
  TARGETS pilz_industrial_motion_planner_testutils
  EXPORT pilz_industrial_motion_planner_testutilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/pilz_industrial_motion_planner_testutils)

install(DIRECTORY include/
        DESTINATION include/pilz_industrial_motion_planner_testutils)

ament_export_targets(pilz_industrial_motion_planner_testutilsTargets
                     HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp moveit_core moveit_msgs tf2_eigen)

ament_package()
