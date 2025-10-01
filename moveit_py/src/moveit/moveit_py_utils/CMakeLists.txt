add_library(moveit_py_utils SHARED src/copy_ros_msg.cpp
                                   src/ros_msg_typecasters.cpp)
target_include_directories(
  moveit_py_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/moveit_py>)
set_target_properties(moveit_py_utils PROPERTIES VERSION
                                                 "${${PROJECT_NAME}_VERSION}")

target_link_libraries(
  moveit_py_utils PUBLIC ${geometry_msgs_TARGETS} ${moveit_msgs_TARGETS}
                         rclcpp::rclcpp pybind11::pybind11)

install(
  TARGETS moveit_py_utils
  EXPORT moveit_py_utilsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/moveit_py)
