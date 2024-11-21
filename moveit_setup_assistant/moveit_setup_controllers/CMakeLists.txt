cmake_minimum_required(VERSION 3.22)
project(moveit_setup_controllers LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(
  MOC_FILES include/moveit_setup_controllers/controller_edit_widget.hpp
  include/moveit_setup_controllers/controllers_widget.hpp
  include/moveit_setup_controllers/urdf_modifications_widget.hpp)

add_library(
  moveit_setup_controllers
  src/control_xacro_config.cpp
  src/controller_edit_widget.cpp
  src/controllers.cpp
  src/controllers_widget.cpp
  src/controllers_config.cpp
  src/modified_urdf_config.cpp
  src/moveit_controllers_config.cpp
  src/moveit_controllers_widget.cpp
  src/ros2_controllers_config.cpp
  src/ros2_controllers_widget.cpp
  src/urdf_modifications.cpp
  src/urdf_modifications_widget.cpp
  ${MOC_FILES})
target_include_directories(
  moveit_setup_controllers
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_controllers>)
ament_target_dependencies(moveit_setup_controllers ament_index_cpp
                          moveit_setup_framework pluginlib rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers moveit_setup_controllers)
endif()

install(DIRECTORY templates DESTINATION share/moveit_setup_controllers)

install(
  TARGETS moveit_setup_controllers
  EXPORT moveit_setup_controllersTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_controllers)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/moveit_setup_controllers)
install(DIRECTORY include/ DESTINATION include/moveit_setup_controllers)
install(DIRECTORY launch DESTINATION share/moveit_setup_controllers)

ament_export_targets(moveit_setup_controllersTargets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
