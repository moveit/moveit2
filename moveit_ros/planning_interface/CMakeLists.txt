cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_planning_interface)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
# find_package(moveit_ros_manipulation REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

find_package(Eigen3 REQUIRED)
# find_package(eigenpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  move_group_interface/include
  robot_interface/include
)

set(THIS_PACKAGE_LIBRARIES
  moveit_common_planning_interface_objects
  moveit_planning_scene_interface
  moveit_move_group_interface
  moveit_robot_interface
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_warehouse
#  moveit_ros_manipulation
  moveit_ros_move_group
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  rclcpp
  rclcpp_action
  Eigen3
  Boost
  pybind11
)

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(py_bindings_tools)
add_subdirectory(common_planning_interface_objects)
add_subdirectory(planning_scene_interface)
add_subdirectory(move_group_interface)
add_subdirectory(rclcpp_interface)
add_subdirectory(robot_interface)
# add_subdirectory(test)

#############
## TESTING ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  # TODO (vatanaksoytezer): Enable once this test is not flaky
  # Move group interface cpp ompl constrained planning integration test
  # ament_add_gtest_executable(move_group_ompl_constraints_test
  #     test/move_group_ompl_constraints_test.cpp
  # )

  # target_link_libraries(move_group_ompl_constraints_test ${THIS_PACKAGE_LIBRARIES})
  # ament_target_dependencies(move_group_ompl_constraints_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  # add_ros_test(test/launch/move_group_ompl_constraints.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # end move group interface cpp ompl constrained planning integration test

endif()

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
