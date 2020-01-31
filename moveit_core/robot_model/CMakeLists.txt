cmake_minimum_required(VERSION 3.5)
set(MOVEIT_LIB_NAME moveit_robot_model)

add_library(${MOVEIT_LIB_NAME} SHARED
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
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  Eigen3
  geometric_shapes
  urdf
  urdfdom_headers
  srdfdom
  visualization_msgs
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_profiler
  moveit_exceptions
  moveit_kinematics_base
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_resources REQUIRED)
  ament_add_gtest(test_robot_model test/test.cpp)
  target_link_libraries(test_robot_model
    moveit_test_utils
    moveit_profiler
    ${Boost_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${rmw_implementation_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    ${MOVEIT_LIB_NAME})
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
