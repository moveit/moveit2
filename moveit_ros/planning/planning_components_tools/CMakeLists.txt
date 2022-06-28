# NOTE: For some reason ament_lint_cmake [linelength] only complains about this file
# TODO(henningkayser): Disable linelength filter once number of digits is configurable - set to 120
# lint_cmake: -linelength
add_executable(moveit_print_planning_model_info src/print_planning_model_info.cpp)
target_link_libraries(moveit_print_planning_model_info moveit_robot_model_loader)
ament_target_dependencies(moveit_print_planning_model_info
    rclcpp
)

add_executable(moveit_print_planning_scene_info src/print_planning_scene_info.cpp)
target_link_libraries(moveit_print_planning_scene_info moveit_planning_scene_monitor)
ament_target_dependencies(moveit_print_planning_scene_info
    rclcpp
)

add_executable(moveit_display_random_state src/display_random_state.cpp)
target_link_libraries(moveit_display_random_state moveit_planning_scene_monitor)
ament_target_dependencies(moveit_display_random_state
    rclcpp
)

add_executable(moveit_visualize_robot_collision_volume src/visualize_robot_collision_volume.cpp)
target_link_libraries(moveit_visualize_robot_collision_volume moveit_planning_scene_monitor)
ament_target_dependencies(moveit_visualize_robot_collision_volume
    rclcpp
    tf2_ros
)

add_executable(moveit_evaluate_collision_checking_speed src/evaluate_collision_checking_speed.cpp)
target_link_libraries(moveit_evaluate_collision_checking_speed moveit_planning_scene_monitor)
ament_target_dependencies(moveit_evaluate_collision_checking_speed
    rclcpp
    Boost
)

if("${catkin_LIBRARIES}" MATCHES "moveit_collision_detection_bullet")
  add_executable(moveit_compare_collision_checking_speed_fcl_bullet src/compare_collision_speed_checking_fcl_bullet.cpp)
  target_link_libraries(moveit_compare_collision_checking_speed_fcl_bullet
    moveit_planning_scene_monitor
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()

add_executable(moveit_publish_scene_from_text src/publish_scene_from_text.cpp)
target_link_libraries(moveit_publish_scene_from_text moveit_planning_scene_monitor moveit_robot_model_loader)
ament_target_dependencies(moveit_publish_scene_from_text
    rclcpp
)

install(TARGETS
  moveit_print_planning_model_info
  moveit_print_planning_scene_info
  moveit_display_random_state
  moveit_visualize_robot_collision_volume
  moveit_evaluate_collision_checking_speed
  moveit_publish_scene_from_text
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# lint_cmake:
