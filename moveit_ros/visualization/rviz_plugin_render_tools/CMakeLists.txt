set(HEADERS
    include/moveit/rviz_plugin_render_tools/octomap_render.hpp
    include/moveit/rviz_plugin_render_tools/planning_link_updater.hpp
    include/moveit/rviz_plugin_render_tools/planning_scene_render.hpp
    include/moveit/rviz_plugin_render_tools/render_shapes.hpp
    include/moveit/rviz_plugin_render_tools/robot_state_visualization.hpp
    include/moveit/rviz_plugin_render_tools/trajectory_visualization.hpp
    include/moveit/rviz_plugin_render_tools/trajectory_panel.hpp
    include/ogre_helpers/mesh_shape.hpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(
  moveit_rviz_plugin_render_tools SHARED
  src/render_shapes.cpp
  src/robot_state_visualization.cpp
  src/planning_scene_render.cpp
  src/planning_link_updater.cpp
  src/octomap_render.cpp
  src/trajectory_visualization.cpp
  src/trajectory_panel.cpp
  src/mesh_shape.cpp
  ${HEADERS})
set_target_properties(moveit_rviz_plugin_render_tools
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(
  moveit_rviz_plugin_render_tools
  PUBLIC Qt5::Widgets
         ${octomap_msgs_TARGETS}
         moveit_core::moveit_collision_detection
         moveit_core::moveit_robot_trajectory
         rclcpp::rclcpp
         rviz_common::rviz_common
         rviz_default_plugins::rviz_default_plugins
         rviz_ogre_vendor::OgreMain)

target_include_directories(moveit_rviz_plugin_render_tools
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
