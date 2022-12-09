target_sources(moveit_core PRIVATE
  src/aabb.cpp
  src/fixed_joint_model.cpp
  src/floating_joint_model.cpp
  src/joint_model.cpp
  src/joint_model_group.cpp
  src/link_model.cpp
  src/planar_joint_model.cpp
  src/prismatic_joint_model.cpp
  src/revolute_joint_model.cpp
  src/robot_model.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/kinematics_base/include> # Work around cyclic dependency between this and kinematics base

)
ament_target_dependencies(moveit_core
  angles
  moveit_msgs
  Eigen3
  geometric_shapes
  urdf
  urdfdom_headers
  srdfdom
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_model test/test.cpp)
  ament_target_dependencies(test_robot_model
    rclcpp
  )
  target_link_libraries(test_robot_model moveit_core)
endif()

install(DIRECTORY include/ DESTINATION include/moveit_core)
