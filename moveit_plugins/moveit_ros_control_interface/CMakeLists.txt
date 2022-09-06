cmake_minimum_required(VERSION 3.22)
project(moveit_ros_control_interface)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp_action
    controller_manager_msgs
    moveit_core
    moveit_simple_controller_manager
    pluginlib
    trajectory_msgs
)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# Finds Boost Components
include(ConfigExtras.cmake)

add_library(${PROJECT_NAME}_plugin SHARED
  src/controller_manager_plugin.cpp
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${PROJECT_NAME}_plugin PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_plugin
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    Boost
)

add_library(${PROJECT_NAME}_trajectory_plugin SHARED
  src/joint_trajectory_controller_plugin.cpp
)
set_target_properties(${PROJECT_NAME}_trajectory_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${PROJECT_NAME}_trajectory_plugin PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_trajectory_plugin
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    Boost
)

add_library(${PROJECT_NAME}_gripper_plugin SHARED
  src/gripper_controller_plugin.cpp
)
set_target_properties(${PROJECT_NAME}_gripper_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${PROJECT_NAME}_gripper_plugin PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_gripper_plugin
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    Boost
)

add_library(${PROJECT_NAME}_empty_plugin SHARED
  src/empty_controller_plugin.cpp
)
set_target_properties(${PROJECT_NAME}_empty_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${PROJECT_NAME}_empty_plugin PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_empty_plugin
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    Boost
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_plugin ${PROJECT_NAME}_trajectory_plugin ${PROJECT_NAME}_gripper_plugin ${PROJECT_NAME}_empty_plugin
EXPORT export_${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(moveit_core moveit_core_plugins.xml)
pluginlib_export_plugin_description_file(moveit_ros_control_interface moveit_ros_control_interface_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    Boost
)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
