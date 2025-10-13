add_library(moveit_exceptions SHARED src/exceptions.cpp)
target_include_directories(
  moveit_exceptions
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
set_target_properties(moveit_exceptions PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_exceptions PUBLIC moveit_utils rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/moveit_core)
