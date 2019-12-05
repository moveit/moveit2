set(MOVEIT_LIB_NAME moveit_robot_trajectory)

add_library(${MOVEIT_LIB_NAME} SHARED src/robot_trajectory.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE  DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
