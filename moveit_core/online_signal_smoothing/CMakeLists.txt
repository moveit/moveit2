# Base class
add_library(moveit_smoothing_base SHARED src/smoothing_base_class.cpp)
target_include_directories(
  moveit_smoothing_base
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(moveit_smoothing_base ${Eigen_LIBRARIES} moveit_macros)
include(GenerateExportHeader)
generate_export_header(moveit_smoothing_base)
set_target_properties(moveit_smoothing_base
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_smoothing_base rclcpp tf2_eigen)

# Plugin implementations
add_library(moveit_acceleration_filter SHARED src/acceleration_filter.cpp)
generate_export_header(moveit_acceleration_filter)
target_include_directories(
  moveit_acceleration_filter
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_acceleration_filter
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
generate_parameter_library(moveit_acceleration_filter_parameters
                           src/acceleration_filter_parameters.yaml)
target_link_libraries(
  moveit_acceleration_filter moveit_acceleration_filter_parameters
  moveit_robot_state moveit_smoothing_base osqp::osqp)
ament_target_dependencies(
  moveit_acceleration_filter srdfdom # include dependency from
                                     # moveit_robot_model
  pluginlib)

add_library(moveit_butterworth_filter SHARED src/butterworth_filter.cpp)
generate_export_header(moveit_butterworth_filter)
target_include_directories(
  moveit_butterworth_filter
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_butterworth_filter
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
generate_parameter_library(moveit_butterworth_filter_parameters
                           src/butterworth_parameters.yaml)
target_link_libraries(
  moveit_butterworth_filter moveit_butterworth_filter_parameters
  moveit_robot_model moveit_smoothing_base)
ament_target_dependencies(
  moveit_butterworth_filter
  srdfdom # include dependency from moveit_robot_model
  pluginlib)

add_library(moveit_ruckig_filter SHARED src/ruckig_filter.cpp)
generate_export_header(moveit_ruckig_filter)
target_include_directories(
  moveit_ruckig_filter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_ruckig_filter
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
generate_parameter_library(moveit_ruckig_filter_parameters
                           src/ruckig_filter_parameters.yaml)
target_link_libraries(
  moveit_ruckig_filter moveit_robot_state moveit_ruckig_filter_parameters
  moveit_smoothing_base ruckig::ruckig)
ament_target_dependencies(
  moveit_ruckig_filter srdfdom # include dependency from moveit_robot_model
  pluginlib)

# Installation
install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_smoothing_base_export.h
        DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_acceleration_filter_export.h
        DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_butterworth_filter_export.h
        DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_ruckig_filter_export.h
        DESTINATION include/moveit_core)

# Testing

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Lowpass filter unit test
  ament_add_gtest(test_butterworth_filter test/test_butterworth_filter.cpp)
  target_link_libraries(test_butterworth_filter moveit_butterworth_filter)

  # Acceleration filter unit test
  ament_add_gtest(test_acceleration_filter test/test_acceleration_filter.cpp)
  target_link_libraries(test_acceleration_filter moveit_acceleration_filter
                        moveit_test_utils)
endif()
