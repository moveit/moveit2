set(MOVEIT_LIB_NAME moveit_planning_scene_rviz_plugin)

add_library(${MOVEIT_LIB_NAME}_core SHARED
  src/planning_scene_display.cpp
  include/moveit/planning_scene_rviz_plugin/planning_scene_display.h)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_rviz_plugin_render_tools rviz_ogre_vendor::OgreMain)
ament_target_dependencies(${MOVEIT_LIB_NAME}_core
    rclcpp
    rviz2
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_msgs
    pluginlib
    Boost
)

add_library(${MOVEIT_LIB_NAME} SHARED src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core  Qt5::Widgets)
ament_target_dependencies(${MOVEIT_LIB_NAME}
    pluginlib
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(
    ${MOVEIT_LIB_NAME}_core
    ${MOVEIT_LIB_NAME}
)
