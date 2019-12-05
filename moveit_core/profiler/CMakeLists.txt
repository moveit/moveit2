set(MOVEIT_LIB_NAME moveit_profiler)

add_library(${MOVEIT_LIB_NAME} SHARED src/profiler.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  rmw_implementation
  urdfdom
  urdfdom_headers
  Boost
  random_numbers
)

install(TARGETS ${MOVEIT_LIB_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
