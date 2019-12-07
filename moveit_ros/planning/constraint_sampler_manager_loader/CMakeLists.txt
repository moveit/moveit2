set(MOVEIT_LIB_NAME moveit_constraint_sampler_manager_loader)

add_library(${MOVEIT_LIB_NAME} SHARED
    src/constraint_sampler_manager_loader.cpp)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  moveit_rdf_loader
  rclcpp
  Boost
  pluginlib
  ament_index_cpp
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
