if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wno-potentially-evaluated-expression)
endif()

add_library(moveit_robot_model_loader SHARED src/robot_model_loader.cpp)
set_target_properties(moveit_robot_model_loader
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_robot_model_loader
  PUBLIC moveit_rdf_loader moveit_kinematics_plugin_loader
         ${moveit_msgs_TARGETS} moveit_core::moveit_collision_detection
         rclcpp::rclcpp urdf::urdf)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
