set(MOVEIT_LIB_NAME moveit_lazy_free_space_updater)

add_library(${MOVEIT_LIB_NAME} SHARED src/lazy_free_space_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
if(APPLE)
  target_link_libraries(${MOVEIT_LIB_NAME} OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  moveit_ros_occupancy_map_monitor
  sensor_msgs
)

install(DIRECTORY include/ DESTINATION include)
