set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME}
  src/iterative_time_parameterization.cpp
  src/iterative_spline_parameterization.cpp
  src/trajectory_tools.cpp
  src/time_optimal_trajectory_generation.cpp
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_robot_state
  moveit_robot_trajectory
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
  Boost)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

#TODO: Fix this so it can be built with colcon
# if(BUILD_TESTING)
#   find_package(moveit_resources REQUIRED)
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#   ament_add_gtest(test_time_parameterization test/test_time_parameterization.cpp)
#   target_link_libraries(test_time_parameterization moveit_test_utils ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
# endif()
