cmake_minimum_required(VERSION 3.10.2)
project(moveit_core VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings
# Remove -Wcast-qual to avoid warnings in tf2/LinearMath/Vector3.h.
# TODO: Add -Wcast-qual back when PR(https://github.com/ros2/geometry2/pull/193) is merged.
add_compile_options(-Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # This too often has false-positives
  add_compile_options(-Wno-maybe-uninitialized)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread iostreams)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBFCL REQUIRED "fcl>=0.5.0")
# replace LIBFCL_LIBRARIES with full path to the library
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

find_package(octomap REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# Set target file path for version.h
set(VERSION_FILE_PATH ${CMAKE_BINARY_DIR}/include)

set(THIS_PACKAGE_INCLUDE_DIRS
    background_processing/include
    exceptions/include
    backtrace/include
    collision_detection/include
    collision_detection_fcl/include
    constraint_samplers/include
    controller_manager/include
    distance_field/include
    collision_distance_field/include
    dynamics_solver/include
    kinematics_base/include
    kinematics_metrics/include
    robot_model/include
    transforms/include
    robot_state/include
    robot_trajectory/include
    kinematic_constraints/include
    macros/include
    planning_interface/include
    planning_request_adapter/include
    planning_scene/include
    profiler/include
    sensor_manager/include
    trajectory_processing/include
    utils/include
)

set(THIS_PACKAGE_LIBRARIES
    moveit_exceptions
    moveit_background_processing
    moveit_kinematics_base
    moveit_robot_model
    moveit_transforms
    moveit_robot_state
    moveit_robot_trajectory
    moveit_planning_interface
    moveit_collision_detection
    moveit_collision_detection_fcl
    moveit_kinematic_constraints
    moveit_planning_scene
    moveit_constraint_samplers
    moveit_planning_request_adapter
    moveit_profiler
    moveit_trajectory_processing
    moveit_distance_field
    moveit_collision_distance_field
    moveit_kinematics_metrics
    moveit_dynamics_solver
    moveit_utils
    moveit_test_utils
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  eigen_stl_containers
  geometric_shapes
  geometry_msgs
  kdl_parser
  moveit_msgs
  octomap_msgs
  random_numbers
  sensor_msgs
  shape_msgs
  srdfdom
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
  Eigen3
  eigen3_cmake_module
  Boost
  OCTOMAP
)
pluginlib_export_plugin_description_file(moveit_core collision_detector_fcl_description.xml)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
  ${VERSION_FILE_PATH}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${random_numbers_INCLUDE_DIRS}
  ${srdfdom_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}
)

# Generate and install version.h
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" MOVEIT_VERSION_MAJOR "${${PROJECT_NAME}_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_MINOR "${${PROJECT_NAME}_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_PATCH "${${PROJECT_NAME}_VERSION}")
set(MOVEIT_VERSION_EXTRA "Alpha")
set(MOVEIT_VERSION "${MOVEIT_VERSION_MAJOR}.${MOVEIT_VERSION_MINOR}.${MOVEIT_VERSION_PATCH}-${MOVEIT_VERSION_EXTRA}")
message(STATUS " *** Building MoveIt ${MOVEIT_VERSION} ***")
configure_file("version/version.h.in" "${VERSION_FILE_PATH}/moveit/version.h")
install(FILES "${VERSION_FILE_PATH}/moveit/version.h" DESTINATION include/moveit)

add_subdirectory(version)
add_subdirectory(macros)
add_subdirectory(backtrace)
add_subdirectory(exceptions)
add_subdirectory(profiler)
add_subdirectory(utils)
add_subdirectory(background_processing)
add_subdirectory(robot_model)
add_subdirectory(collision_detection)
add_subdirectory(kinematics_base)
add_subdirectory(transforms)
add_subdirectory(robot_state)
add_subdirectory(collision_detection_fcl)
add_subdirectory(robot_trajectory)
add_subdirectory(kinematic_constraints)
add_subdirectory(trajectory_processing)
add_subdirectory(planning_scene)
add_subdirectory(controller_manager)
add_subdirectory(sensor_manager)
add_subdirectory(constraint_samplers)
add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)
add_subdirectory(distance_field)
add_subdirectory(collision_distance_field)
add_subdirectory(kinematics_metrics)
add_subdirectory(dynamics_solver)

ament_export_include_directories(include)
ament_export_libraries(${THIS_PACKAGE_LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
