set(MOVEIT_LIB_NAME moveit_constraint_sampler_manager_loader)

add_library(${MOVEIT_LIB_NAME} SHARED
    src/constraint_sampler_manager_loader.cpp)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  rclcpp
  Boost
  pluginlib
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_rdf_loader
)

install(DIRECTORY include/ DESTINATION include)
