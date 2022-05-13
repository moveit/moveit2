cmake_minimum_required(VERSION 3.10.2)
project(moveit_core LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.5.0")
# replace LIBFCL_LIBRARIES with full paths to the libraries
set(LIBFCL_LIBRARIES_FULL "")
foreach(LIBFCL_LIBRARY ${LIBFCL_LIBRARIES})
  find_library(${LIBFCL_LIBRARY}_LIB ${LIBFCL_LIBRARY} ${LIBFCL_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES_FULL ${${LIBFCL_LIBRARY}_LIB})
endforeach()
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

find_package(Bullet 2.87 REQUIRED)
find_package(angles REQUIRED)
find_package(OCTOMAP REQUIRED)
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
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
# TODO: Port python bindings
# find_package(pybind11 REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

# Set target file path for version.h
set(VERSION_FILE_PATH ${CMAKE_BINARY_DIR}/include)

set(THIS_PACKAGE_INCLUDE_DIRS
    exceptions/include
    collision_detection/include
    collision_detection_fcl/include
    collision_detection_bullet/include
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
    online_signal_smoothing/include
    planning_interface/include
    planning_request_adapter/include
    planning_scene/include
    # TODO: Port python bindings
    # python/tools/include
    sensor_manager/include
    trajectory_processing/include
    utils/include
)

set(THIS_PACKAGE_LIBRARIES
    moveit_butterworth_filter
    moveit_collision_distance_field
    moveit_collision_detection
    moveit_collision_detection_fcl
    moveit_collision_detection_bullet
    moveit_dynamics_solver
    moveit_constraint_samplers
    moveit_distance_field
    moveit_exceptions
    moveit_kinematics_base
    moveit_kinematic_constraints
    moveit_kinematics_metrics
    moveit_planning_interface
    moveit_planning_scene
    moveit_planning_request_adapter
    # TODO: Port python bindings
    # moveit_python_tools
    moveit_robot_model
    moveit_robot_state
    moveit_robot_trajectory
    moveit_smoothing_base
    moveit_test_utils
    moveit_trajectory_processing
    moveit_transforms
    moveit_utils
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
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
  OCTOMAP
  Bullet
  ruckig
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS}
                    ${VERSION_FILE_PATH})

# Generate and install version.h
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" MOVEIT_VERSION_MAJOR "${${PROJECT_NAME}_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_MINOR "${${PROJECT_NAME}_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_PATCH "${${PROJECT_NAME}_VERSION}")
set(MOVEIT_VERSION_EXTRA "Alpha")
set(MOVEIT_VERSION "${MOVEIT_VERSION_MAJOR}.${MOVEIT_VERSION_MINOR}.${MOVEIT_VERSION_PATCH}-${MOVEIT_VERSION_EXTRA}")
message(STATUS " *** Building MoveIt ${MOVEIT_VERSION} ***")
configure_file("version/version.h.in" "${VERSION_FILE_PATH}/moveit/version.h")
install(FILES "${VERSION_FILE_PATH}/moveit/version.h" DESTINATION include/moveit)

add_subdirectory(collision_distance_field)
add_subdirectory(constraint_samplers)
add_subdirectory(controller_manager)
add_subdirectory(distance_field)
add_subdirectory(dynamics_solver)
add_subdirectory(exceptions)
add_subdirectory(kinematics_base)
add_subdirectory(kinematic_constraints)
add_subdirectory(kinematics_metrics)
add_subdirectory(macros)
add_subdirectory(online_signal_smoothing)
add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)
add_subdirectory(planning_scene)
add_subdirectory(robot_model)
add_subdirectory(robot_state)
add_subdirectory(robot_trajectory)
add_subdirectory(sensor_manager)
add_subdirectory(trajectory_processing)
add_subdirectory(transforms)
add_subdirectory(utils)
add_subdirectory(version)

add_subdirectory(collision_detection)
add_subdirectory(collision_detection_bullet)
add_subdirectory(collision_detection_fcl)

# TODO: Port python bindings
# add_subdirectory(python)
# set(pymoveit_libs
#   moveit_collision_detection
#   moveit_kinematic_constraints
#   moveit_planning_scene
#   moveit_python_tools
#   moveit_robot_model
#   moveit_robot_state
#   moveit_transforms
# )
#
# pybind_add_module(pymoveit_core
#     python/pymoveit_core.cpp
#     collision_detection/src/pycollision_detection.cpp
#     robot_model/src/pyrobot_model.cpp
#     robot_state/src/pyrobot_state.cpp
#     transforms/src/pytransforms.cpp
#     planning_scene/src/pyplanning_scene.cpp
#     kinematic_constraints/src/pykinematic_constraint.cpp
# )
# target_include_directories(pymoveit_core SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
# target_link_libraries(pymoveit_core PRIVATE ${pymoveit_libs} ${catkin_LIBRARIES})
#
# #catkin_lint: ignore_once undefined_target (pymoveit_core is defined by pybind_add_module)
# install(TARGETS pymoveit_core LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})


install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} orocos_kdl_vendor)

# Plugin exports
pluginlib_export_plugin_description_file(moveit_core collision_detector_fcl_description.xml)
pluginlib_export_plugin_description_file(moveit_core collision_detector_bullet_description.xml)
pluginlib_export_plugin_description_file(moveit_core filter_plugin_butterworth.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
