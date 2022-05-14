cmake_minimum_required(VERSION 3.10.2)
project(moveit_setup_assistant)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(ompl REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_visualization
  rclcpp
  urdf
  srdfdom
)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
#set(HEADERS
#  src/widgets/navigation_widget.h
#  src/widgets/header_widget.h
#  src/widgets/setup_assistant_widget.h
#  src/widgets/start_screen_widget.h
#  src/widgets/planning_groups_widget.h
#  src/widgets/double_list_widget.h
#  src/widgets/kinematic_chain_widget.h
#  src/widgets/group_edit_widget.h
#  src/widgets/default_collisions_widget.h
#  src/widgets/robot_poses_widget.h
#  src/widgets/end_effectors_widget.h
#  src/widgets/virtual_joints_widget.h
#  src/widgets/passive_joints_widget.h
#  src/widgets/perception_widget.h
#  src/widgets/ros_controllers_widget.h
#  src/widgets/controller_edit_widget.h
#  src/widgets/configuration_files_widget.h
#  src/widgets/setup_screen_widget.h
#  src/widgets/simulation_widget.h
#  src/widgets/author_information_widget.h
#)

# Main Widgets Library - all screens (navigation options)
#add_library(${PROJECT_NAME}_widgets
#  src/widgets/start_screen_widget.cpp
#  src/widgets/planning_groups_widget.cpp
#  src/widgets/double_list_widget.cpp
#  src/widgets/kinematic_chain_widget.cpp
#  src/widgets/group_edit_widget.cpp
#  src/widgets/default_collisions_widget.cpp
#  src/widgets/robot_poses_widget.cpp
#  src/widgets/end_effectors_widget.cpp
#  src/widgets/virtual_joints_widget.cpp
#  src/widgets/passive_joints_widget.cpp
#  src/widgets/perception_widget.cpp
#  src/widgets/configuration_files_widget.cpp
#  src/widgets/ros_controllers_widget.cpp
#  src/widgets/controller_edit_widget.cpp
#  src/widgets/navigation_widget.cpp
#  src/widgets/header_widget.cpp
#  src/widgets/setup_assistant_widget.cpp
#  src/widgets/setup_screen_widget.cpp
#  src/widgets/simulation_widget.cpp
#  src/widgets/author_information_widget.cpp
#  ${HEADERS}
#)

# Tools Library
add_library(${PROJECT_NAME}_tools
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
  src/tools/xml_manipulation.cpp
  src/tools/collision_linear_model.cpp
  src/tools/collision_matrix_model.cpp
  src/tools/rotated_header_view.cpp
  src/tools/xml_syntax_highlighter.cpp
)
target_include_directories(${PROJECT_NAME}_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(
  ${PROJECT_NAME}_tools
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(${PROJECT_NAME}_updater src/collisions_updater.cpp)
target_include_directories(${PROJECT_NAME}_updater PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME}_updater
  ${PROJECT_NAME}_tools
)

ament_target_dependencies(
  ${PROJECT_NAME}_updater
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

set_target_properties(
  ${PROJECT_NAME}_updater
  PROPERTIES OUTPUT_NAME collisions_updater
  PREFIX ""
)

install(
  TARGETS
    ${PROJECT_NAME}_tools
    ${PROJECT_NAME}_updater
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY templates DESTINATION share/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Unit tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
