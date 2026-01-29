add_library(
  moveit_mesh_filter SHARED
  src/mesh_filter_base.cpp src/sensor_model.cpp src/stereo_camera_model.cpp
  src/gl_renderer.cpp src/gl_mesh.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_mesh_filter)
target_include_directories(
  moveit_mesh_filter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_mesh_filter
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(moveit_mesh_filter ${GL_LIBS} ${SYSTEM_GL_LIBRARIES}
                        geometric_shapes::geometric_shapes
                        moveit_core::moveit_macros)



install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_mesh_filter_export.h
        DESTINATION include/moveit_ros_perception)
