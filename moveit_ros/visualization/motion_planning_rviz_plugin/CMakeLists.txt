set(HEADERS
    include/moveit/motion_planning_rviz_plugin/motion_planning_display.h
    include/moveit/motion_planning_rviz_plugin/motion_planning_frame.h
    include/moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.h
    include/moveit/motion_planning_rviz_plugin/motion_planning_param_widget.h
    include/moveit/motion_planning_rviz_plugin/interactive_marker_display.h)
qt5_wrap_ui(UIC_FILES src/ui/motion_planning_rviz_plugin_frame.ui
            src/ui/motion_planning_rviz_plugin_frame_joints.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
    src/motion_planning_frame.cpp
    src/motion_planning_frame_context.cpp
    src/motion_planning_frame_planning.cpp
    src/motion_planning_frame_objects.cpp
    src/motion_planning_frame_scenes.cpp
    src/motion_planning_frame_states.cpp
    src/motion_planning_frame_joints_widget.cpp
    src/motion_planning_display.cpp
    src/motion_planning_frame_manipulation.cpp
    src/motion_planning_param_widget.cpp
    src/interactive_marker_display.cpp
    src/icons/icons.qrc)

add_library(moveit_motion_planning_rviz_plugin_core SHARED
            ${SOURCE_FILES} ${HEADERS} ${UIC_FILES})
set_target_properties(moveit_motion_planning_rviz_plugin_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_motion_planning_rviz_plugin_core moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin)
ament_target_dependencies(
  moveit_motion_planning_rviz_plugin_core
  moveit_ros_robot_interaction
  moveit_ros_planning_interface
  moveit_ros_warehouse
  rviz2
  rviz_ogre_vendor
  Qt5
  pluginlib)
target_include_directories(moveit_motion_planning_rviz_plugin_core
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(moveit_motion_planning_rviz_plugin SHARED src/plugin_init.cpp)
set_target_properties(moveit_motion_planning_rviz_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_motion_planning_rviz_plugin
                      moveit_motion_planning_rviz_plugin_core)
ament_target_dependencies(
  moveit_motion_planning_rviz_plugin moveit_ros_robot_interaction
  moveit_ros_warehouse pluginlib rviz_ogre_vendor)
target_include_directories(moveit_motion_planning_rviz_plugin
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
