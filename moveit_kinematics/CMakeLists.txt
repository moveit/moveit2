cmake_minimum_required(VERSION 3.22)
project(moveit_kinematics)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(random_numbers REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DIRS
  kdl_kinematics_plugin/include
  lma_kinematics_plugin/include
  srv_kinematics_plugin/include
  cached_ik_kinematics_plugin/include
)

set(THIS_PACKAGE_LIBRARIES
  moveit_cached_ik_kinematics_base
  moveit_cached_ik_kinematics_plugin
  moveit_kdl_kinematics_plugin
  moveit_lma_kinematics_plugin
  moveit_srv_kinematics_plugin
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  moveit_core
  moveit_ros_planning
  Boost
)

pluginlib_export_plugin_description_file(moveit_core kdl_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core lma_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core srv_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core cached_ik_kinematics_plugin_description.xml)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(cached_ik_kinematics_plugin)
add_subdirectory(ikfast_kinematics_plugin)
add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(lma_kinematics_plugin)
add_subdirectory(srv_kinematics_plugin)
add_subdirectory(test)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
