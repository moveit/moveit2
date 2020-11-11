cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_visualization)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # suppress warning in Ogre
  add_compile_options(-Wno-deprecated-register)
endif()

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

find_package(Boost REQUIRED thread date_time system filesystem)
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# TODO(JafarAbdi): Uncomment when porting each package
# find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_robot_interaction REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# Qt Stuff
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

# catkin_install_python(PROGRAMS scripts/moveit_joy.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# catkin_python_setup()

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    planning_scene_rviz_plugin/include
                    motion_planning_rviz_plugin/include
                    trajectory_rviz_plugin/include
)

add_subdirectory(rviz_plugin_render_tools)
add_subdirectory(robot_state_rviz_plugin)
add_subdirectory(planning_scene_rviz_plugin)
add_subdirectory(motion_planning_rviz_plugin)
add_subdirectory(trajectory_rviz_plugin)

install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common motion_planning_rviz_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common trajectory_rviz_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common planning_scene_rviz_plugin_description.xml)
pluginlib_export_plugin_description_file(rviz_common robot_state_rviz_plugin_description.xml)

#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/moveit_joy.test)
#endif()

ament_export_dependencies(class_loader)
ament_export_dependencies(geometric_shapes)
ament_export_dependencies(interactive_markers)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_planning_interface)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(moveit_core)
ament_export_dependencies(octomap_msgs)
ament_export_dependencies(moveit_ros_robot_interaction)
ament_export_dependencies(object_recognition_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclpy)
ament_export_dependencies(rviz2)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(Eigen3)
ament_export_dependencies(rviz_ogre_vendor)
ament_package()

include_directories("${OGRE_PREFIX_DIR}/include")
