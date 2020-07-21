set(MOVEIT_LIB_NAME moveit_robot_model_loader)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  add_compile_options(-Wno-potentially-evaluated-expression)
endif()

add_library(${MOVEIT_LIB_NAME} SHARED src/robot_model_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  urdf
  Boost
  moveit_core
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_rdf_loader
  moveit_kinematics_plugin_loader
)

install(DIRECTORY include/ DESTINATION include)
