if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wno-potentially-evaluated-expression)
endif()

add_library(moveit_robot_model_loader SHARED src/robot_model_loader.cpp)
set_target_properties(moveit_robot_model_loader
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_robot_model_loader
  moveit_rdf_loader
  moveit_kinematics_plugin_loader
  rclcpp::rclcpp
  urdf::urdf
  Boost::headers
  moveit_core::moveit_core
  ${moveit_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
