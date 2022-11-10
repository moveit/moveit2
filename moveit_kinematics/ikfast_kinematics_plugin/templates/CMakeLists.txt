cmake_minimum_required(VERSION 3.22)
project(_PACKAGE_NAME_)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(LAPACK REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)

generate_parameter_library(
  ikfast_kinematics_parameters # cmake target name for the parameter library
  src/ikfast_kinematics_parameters.yaml # path to input yaml file
)

set(IKFAST_LIBRARY_NAME _LIBRARY_NAME_)
add_library(${IKFAST_LIBRARY_NAME} SHARED src/_ROBOT_NAME___GROUP_NAME__ikfast_moveit_plugin.cpp)
ament_target_dependencies(${IKFAST_LIBRARY_NAME}
  rclcpp
  moveit_core
  pluginlib
  tf2_kdl
  orocos_kdl
  tf2_eigen
)
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable -Wno-unused-parameter)

target_link_libraries(${IKFAST_LIBRARY_NAME}
  ikfast_kinematics_parameters
)

install(TARGETS ${IKFAST_LIBRARY_NAME} ikfast_kinematics_parameters
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(moveit_core _ROBOT_NAME___GROUP_NAME__moveit_ikfast_plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_kdl)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(generate_parameter_library)
ament_package()
