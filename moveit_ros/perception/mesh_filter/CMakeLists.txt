set(MOVEIT_LIB_NAME moveit_mesh_filter)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/mesh_filter_base.cpp
  src/sensor_model.cpp
  src/stereo_camera_model.cpp
  src/gl_renderer.cpp
  src/gl_mesh.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  moveit_core
  geometric_shapes
  Eigen3
  Boost
)
target_link_libraries(${MOVEIT_LIB_NAME} ${gl_LIBS} GLUT::GLUT ${GLEW_LIBRARIES})

# TODO: enable testing
# if(CATKIN_ENABLE_TESTING)
#   #catkin_lint: ignore_once env_var
#   # Can only run this test if we have a display
#   if(DEFINED ENV{DISPLAY} AND NOT $ENV{DISPLAY} STREQUAL "")
#     catkin_add_gtest(mesh_filter_test test/mesh_filter_test.cpp)
#     target_link_libraries(mesh_filter_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_mesh_filter)
#   else()
#     message("No display, will not configure tests for moveit_ros_perception/mesh_filter")
#   endif()
# endif()

install(DIRECTORY include/ DESTINATION include)
