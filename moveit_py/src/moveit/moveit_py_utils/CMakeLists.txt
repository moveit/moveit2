add_library(moveit_py_utils SHARED src/copy_ros_msg.cpp
                                   src/ros_msg_typecasters.cpp)
target_include_directories(
  moveit_py_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/moveit_py>)
set_target_properties(moveit_py_utils PROPERTIES VERSION
                                                 "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_py_utils rclcpp moveit_msgs geometry_msgs
                          pybind11)

install(
  TARGETS moveit_py_utils
  EXPORT moveit_py_utilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/moveit_py)
