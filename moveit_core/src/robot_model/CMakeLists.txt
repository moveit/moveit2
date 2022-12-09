target_sources(moveit_core PRIVATE
  aabb.cpp
  fixed_joint_model.cpp
  floating_joint_model.cpp
  joint_model.cpp
  joint_model_group.cpp
  link_model.cpp
  planar_joint_model.cpp
  prismatic_joint_model.cpp
  revolute_joint_model.cpp
  robot_model.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_model test/test.cpp)
  ament_target_dependencies(test_robot_model
    rclcpp
  )
  target_link_libraries(test_robot_model moveit::core)
endif()
