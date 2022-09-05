cmake_minimum_required(VERSION 3.22)
project(moveit_setup_srdf_plugins)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(pluginlib REQUIRED)

qt5_wrap_cpp(MOC_FILES
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
    include/moveit_setup_srdf_plugins/virtual_joints_widget.hpp
)

add_library(${PROJECT_NAME}
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
    ${MOC_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  moveit_setup_framework
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_srdf test/test_srdf.cpp)
  target_link_libraries(test_srdf ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(moveit_setup_framework moveit_setup_framework_plugins.xml)

ament_package()
