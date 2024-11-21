cmake_minimum_required(VERSION 3.22)
project(moveit_kinematics LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(class_loader REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DIRS
    kdl_kinematics_plugin/include srv_kinematics_plugin/include
    cached_ik_kinematics_plugin/include)

set(THIS_PACKAGE_LIBRARIES
    cached_ik_kinematics_parameters
    moveit_cached_ik_kinematics_base
    moveit_cached_ik_kinematics_plugin
    kdl_kinematics_parameters
    moveit_kdl_kinematics_plugin
    srv_kinematics_parameters
    moveit_srv_kinematics_plugin)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    Eigen3
    class_loader
    generate_parameter_library
    kdl_parser
    moveit_core
    moveit_msgs
    moveit_ros_planning
    orocos_kdl_vendor
    pluginlib
    random_numbers
    rclcpp
    tf2_kdl)

pluginlib_export_plugin_description_file(moveit_core
                                         kdl_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core
                                         srv_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(
  moveit_core cached_ik_kinematics_plugin_description.xml)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(cached_ik_kinematics_plugin)
add_subdirectory(ikfast_kinematics_plugin)
add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(srv_kinematics_plugin)
add_subdirectory(test)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_kinematics)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
