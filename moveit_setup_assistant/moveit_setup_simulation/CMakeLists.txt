cmake_minimum_required(VERSION 3.22)
project(moveit_setup_simulation LANGUAGES CXX)

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

qt5_wrap_cpp(MOC_FILES include/moveit_setup_simulation/simulation_widget.hpp)

add_library(moveit_setup_simulation src/simulation.cpp
                                    src/simulation_widget.cpp ${MOC_FILES})
target_include_directories(
  moveit_setup_simulation
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_simulation>)
ament_target_dependencies(moveit_setup_simulation ament_index_cpp
                          moveit_setup_framework pluginlib rclcpp)

install(DIRECTORY templates DESTINATION share/moveit_setup_simulation)

install(
  TARGETS moveit_setup_simulation
  EXPORT moveit_setup_simulationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_simulation)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/moveit_setup_simulation)
install(DIRECTORY include/ DESTINATION include/moveit_setup_simulation)

ament_export_targets(moveit_setup_simulationTargets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
