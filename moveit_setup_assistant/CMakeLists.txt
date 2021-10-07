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
# find_package(rosconsole REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)

find_package(ompl REQUIRED)

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS
  src/widgets/navigation_widget.h
  src/widgets/header_widget.h
  src/widgets/setup_assistant_widget.h
  src/widgets/start_screen_widget.h
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
  src/widgets/setup_screen_widget.h
#  src/widgets/simulation_widget.h
#  src/widgets/author_information_widget.h
)

# Tools Library
add_library(${PROJECT_NAME}_tools
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
#  src/tools/collision_linear_model.cpp
#  src/tools/collision_matrix_model.cpp
#  src/tools/rotated_header_view.cpp
)
target_include_directories(${PROJECT_NAME}_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(
  ${PROJECT_NAME}_tools
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_visualization"
  "rviz2"
  "rviz_common"
  "rclcpp"
  "urdf"
  "srdfdom"
)

# Main Widgets Library - all screens (navigation options)
add_library(${PROJECT_NAME}_widgets
  src/widgets/start_screen_widget.cpp
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
  src/widgets/navigation_widget.cpp
  src/widgets/header_widget.cpp
  src/widgets/setup_assistant_widget.cpp
  src/widgets/setup_screen_widget.cpp
  ${HEADERS}
)
target_link_libraries(${PROJECT_NAME}_widgets
    ${PROJECT_NAME}_tools
)
target_include_directories(${PROJECT_NAME}_widgets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(${PROJECT_NAME}_widgets PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(
  ${PROJECT_NAME}_widgets
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_visualization"
  "rviz2"
  "rviz_common"
  "rclcpp"
  "urdf"
  "srdfdom"
)

# executables
add_executable(${PROJECT_NAME} src/setup_assistant_main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_visualization"
  "rviz2"
  "rviz_common"
  "rclcpp"
  "urdf"
  "srdfdom"
)

add_executable(${PROJECT_NAME}_updater src/collisions_updater.cpp)
target_include_directories(${PROJECT_NAME}_updater PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_updater
  ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools
)

ament_target_dependencies(
  ${PROJECT_NAME}_updater
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_visualization"
  "rviz2"
  "rviz_common"
  "rclcpp"
  "urdf"
  "srdfdom"
)

set_target_properties(
  ${PROJECT_NAME}_updater
  PROPERTIES OUTPUT_NAME collisions_updater
  PREFIX ""
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_updater
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})
install(DIRECTORY templates DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_widgets ${PROJECT_NAME}_tools
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Unit tests
if(BUILD_TESTING)
  # catkin_add_gtest(test_reading_writing_config test/moveit_config_data_test.cpp)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
