set(MOVEIT_LIB_NAME moveit_rdf_loader)

add_library(${MOVEIT_LIB_NAME} SHARED src/rdf_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
      rclcpp
      ament_index_cpp
      urdf
      srdfdom
      moveit_core
      Boost)

install(DIRECTORY include/ DESTINATION include)
