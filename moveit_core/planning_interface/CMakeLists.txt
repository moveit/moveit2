set(MOVEIT_LIB_NAME moveit_planning_interface)

add_library(${MOVEIT_LIB_NAME}
  src/planning_interface.cpp
  src/planning_response.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state moveit_robot_trajectory ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

ament_target_dependencies(${MOVEIT_LIB_NAME}
  "moveit_robot_state"
  "moveit_robot_trajectory"
  "rclcpp"
  "rmw_implementation"
  "urdfdom"
  "urdfdom_headers"
  "boost"
)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
