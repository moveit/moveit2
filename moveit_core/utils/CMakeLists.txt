set(MOVEIT_LIB_NAME moveit_utils)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/lexical_casts.cpp
  src/message_checks.cpp
  src/rclcpp_utils.cpp
)
<<<<<<< HEAD
ament_target_dependencies(${MOVEIT_LIB_NAME} Boost moveit_msgs)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
=======
target_include_directories(moveit_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
ament_target_dependencies(moveit_utils Boost moveit_msgs rclcpp fmt)
target_link_libraries(moveit_utils rsl::rsl)
set_target_properties(moveit_utils PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
>>>>>>> 3df6b2813 (Depend on `rsl::rsl` as a non-Ament dependency (#2578))

install(DIRECTORY include/ DESTINATION include)


find_package(ament_index_cpp REQUIRED)
<<<<<<< HEAD
set(MOVEIT_TEST_LIB_NAME moveit_test_utils)
add_library(${MOVEIT_TEST_LIB_NAME} SHARED src/robot_model_test_utils.cpp)
target_link_libraries(${MOVEIT_TEST_LIB_NAME} moveit_robot_model)
ament_target_dependencies(${MOVEIT_TEST_LIB_NAME}
=======
add_library(moveit_test_utils SHARED src/robot_model_test_utils.cpp)
target_include_directories(moveit_test_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
target_link_libraries(moveit_test_utils moveit_robot_model moveit_kinematics_base rsl::rsl)
ament_target_dependencies(moveit_test_utils
>>>>>>> 3df6b2813 (Depend on `rsl::rsl` as a non-Ament dependency (#2578))
  ament_index_cpp
  Boost
  geometry_msgs
  urdf
  srdfdom
  urdfdom
  urdfdom_headers
<<<<<<< HEAD
=======
  rclcpp
  fmt
>>>>>>> 3df6b2813 (Depend on `rsl::rsl` as a non-Ament dependency (#2578))
)
set_target_properties(${MOVEIT_TEST_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
