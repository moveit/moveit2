cmake_minimum_required(VERSION 3.5)
project(moveit_py)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

add_subdirectory(src/moveit/moveit_py_utils)

ament_python_install_package(moveit)

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/moveit"
  )
endfunction()

pybind11_add_module(core
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
	src/moveit/moveit_core/robot_trajectory/robot_trajectory.cpp
	)
target_link_libraries(core PRIVATE moveit_ros_planning::moveit_cpp
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

pybind11_add_module(planning
	src/moveit/planning.cpp
        src/moveit/moveit_ros/moveit_cpp/moveit_cpp.cpp
	src/moveit/moveit_ros/moveit_cpp/planning_component.cpp
	src/moveit/moveit_ros/planning_scene_monitor/planning_scene_monitor.cpp
	)
target_link_libraries(planning PRIVATE moveit_ros_planning::moveit_cpp
				       moveit_ros_planning::moveit_planning_scene_monitor
				       moveit_core::moveit_utils
				       rclcpp::rclcpp
				       moveit_py_utils)
configure_build_install_location(planning)

ament_package()
