cmake_minimum_required(VERSION 3.22)
project(moveit_setup_srdf_plugins LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(pluginlib REQUIRED)

qt5_wrap_cpp(
  MOC_FILES
  include/moveit_setup_srdf_plugins/collision_linear_model.hpp
  include/moveit_setup_srdf_plugins/collision_matrix_model.hpp
  include/moveit_setup_srdf_plugins/default_collisions_widget.hpp
  include/moveit_setup_srdf_plugins/end_effectors_widget.hpp
  include/moveit_setup_srdf_plugins/group_edit_widget.hpp
  include/moveit_setup_srdf_plugins/kinematic_chain_widget.hpp
  include/moveit_setup_srdf_plugins/passive_joints_widget.hpp
  include/moveit_setup_srdf_plugins/planning_groups_widget.hpp
  include/moveit_setup_srdf_plugins/robot_poses_widget.hpp
  include/moveit_setup_srdf_plugins/rotated_header_view.hpp
  include/moveit_setup_srdf_plugins/virtual_joints_widget.hpp)

add_library(
  moveit_setup_srdf_plugins
  src/collision_linear_model.cpp
  src/collision_matrix_model.cpp
  src/compute_default_collisions.cpp
  src/default_collisions.cpp
  src/default_collisions_widget.cpp
  src/end_effectors.cpp
  src/end_effectors_widget.cpp
  src/group_edit_widget.cpp
  src/group_meta_config.cpp
  src/kinematic_chain_widget.cpp
  src/passive_joints.cpp
  src/passive_joints_widget.cpp
  src/planning_groups.cpp
  src/planning_groups_widget.cpp
  src/robot_poses.cpp
  src/robot_poses_widget.cpp
  src/rotated_header_view.cpp
  src/virtual_joints.cpp
  src/virtual_joints_widget.cpp
  ${MOC_FILES})
target_include_directories(
  moveit_setup_srdf_plugins
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_srdf_plugins>)
ament_target_dependencies(moveit_setup_srdf_plugins moveit_setup_framework
                          pluginlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_srdf test/test_srdf.cpp)
  target_link_libraries(test_srdf moveit_setup_srdf_plugins)
endif()

install(
  TARGETS moveit_setup_srdf_plugins
  EXPORT moveit_setup_srdf_pluginsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_srdf_plugins)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/moveit_setup_srdf_plugins)
install(DIRECTORY include/ DESTINATION include/moveit_setup_srdf_plugins)

ament_export_targets(moveit_setup_srdf_pluginsTargets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
