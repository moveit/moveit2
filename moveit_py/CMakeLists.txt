cmake_minimum_required(VERSION 3.22)
project(moveit_py)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# enables using the Python extensions from the build space for testing
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_moveit/__init__.py" "")

add_subdirectory(src/moveit/moveit_py_utils)

ament_python_install_package(moveit)

# Set the build location and install location for a CPython extension
function(configure_build_install_location library_name)
  # Install into test_moveit folder in build space for unit tests to import
  set_target_properties(
    ${library_name}
    PROPERTIES # Use generator expression to avoid prepending a build type
               # specific directory on Windows
               LIBRARY_OUTPUT_DIRECTORY
               $<1:${CMAKE_CURRENT_BINARY_DIR}/test_moveit>
               RUNTIME_OUTPUT_DIRECTORY
               $<1:${CMAKE_CURRENT_BINARY_DIR}/test_moveit>)

  install(TARGETS ${library_name} DESTINATION "${PYTHON_INSTALL_DIR}/moveit")
endfunction()

pybind11_add_module(
  core
  src/moveit/core.cpp
  src/moveit/moveit_core/collision_detection/collision_common.cpp
  src/moveit/moveit_core/collision_detection/collision_matrix.cpp
  src/moveit/moveit_core/collision_detection/world.cpp
  src/moveit/moveit_core/controller_manager/controller_manager.cpp
  src/moveit/moveit_core/kinematic_constraints/utils.cpp
  src/moveit/moveit_core/planning_interface/planning_response.cpp
  src/moveit/moveit_core/planning_scene/planning_scene.cpp
  src/moveit/moveit_core/transforms/transforms.cpp
  src/moveit/moveit_core/robot_model/joint_model.cpp
  src/moveit/moveit_core/robot_model/joint_model_group.cpp
  src/moveit/moveit_core/robot_model/robot_model.cpp
  src/moveit/moveit_core/robot_state/robot_state.cpp
  src/moveit/moveit_core/robot_trajectory/robot_trajectory.cpp)
target_link_libraries(
  core
  PRIVATE moveit_ros_planning::moveit_cpp
          rclcpp::rclcpp
          moveit_core::moveit_transforms
          moveit_core::moveit_kinematic_constraints
          moveit_core::moveit_planning_interface
          moveit_core::moveit_planning_scene
          moveit_core::moveit_utils
          moveit_core::moveit_robot_model
          moveit_core::moveit_robot_state
          moveit_py_utils)
configure_build_install_location(core)

pybind11_add_module(
  planning
  src/moveit/planning.cpp
  src/moveit/moveit_ros/moveit_cpp/moveit_cpp.cpp
  src/moveit/moveit_ros/moveit_cpp/planning_component.cpp
  src/moveit/moveit_ros/planning_scene_monitor/planning_scene_monitor.cpp
  src/moveit/moveit_ros/trajectory_execution_manager/trajectory_execution_manager.cpp
)
target_link_libraries(
  planning
  PRIVATE moveit_ros_planning::moveit_cpp
          moveit_ros_planning::moveit_planning_scene_monitor
          moveit_ros_planning::moveit_trajectory_execution_manager
          moveit_core::moveit_utils
          rclcpp::rclcpp
          moveit_py_utils)
configure_build_install_location(planning)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests test/unit/test_robot_model.py test/unit/test_robot_state.py)
  foreach(test_path ${_pytest_tests})
    get_filename_component(_test_name ${test_path} NAME_WE)
    ament_add_pytest_test(
      ${_test_name}
      ${test_path}
      APPEND_ENV
      AMENT_PREFIX_INDEX=${ament_index_build_path}
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT
      60
      WORKING_DIRECTORY
      "${CMAKE_SOURCE_DIR}")
  endforeach()
endif()

ament_export_targets(moveit_py_utilsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_ros_planning_interface)
ament_package()
