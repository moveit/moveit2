set(MOVEIT_LIB_NAME moveit_utils)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/lexical_casts.cpp
  src/message_checks.cpp
  src/rclcpp_utils.cpp
)
ament_target_dependencies(${MOVEIT_LIB_NAME} Boost moveit_msgs)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(DIRECTORY include/ DESTINATION include)


find_package(ament_index_cpp REQUIRED)
set(MOVEIT_TEST_LIB_NAME moveit_test_utils)
add_library(${MOVEIT_TEST_LIB_NAME} SHARED src/robot_model_test_utils.cpp)
target_link_libraries(${MOVEIT_TEST_LIB_NAME} moveit_robot_model)
ament_target_dependencies(${MOVEIT_TEST_LIB_NAME}
  ament_index_cpp
  Boost
  geometry_msgs
  urdf
  srdfdom
  urdfdom
  urdfdom_headers
)
set_target_properties(${MOVEIT_TEST_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
