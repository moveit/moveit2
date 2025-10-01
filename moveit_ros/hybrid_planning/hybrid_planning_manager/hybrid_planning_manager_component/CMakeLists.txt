# Hybrid planning manager component
generate_parameter_library(hp_manager_parameters res/hp_manager_parameters.yaml)
add_library(moveit_hybrid_planning_manager SHARED
            src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_hybrid_planning_manager
  PUBLIC hp_manager_parameters rclcpp_action::rclcpp_action
         rclcpp_components::component moveit_core::moveit_utils
         pluginlib::pluginlib ${moveit_msgs_TARGETS})
