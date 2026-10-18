# Base class
add_library(moveit_smoothing_base SHARED src/smoothing_base_class.cpp)
target_include_directories(
  moveit_smoothing_base
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(moveit_smoothing_base ${Eigen_LIBRARIES} moveit_macros
                      rclcpp::rclcpp tf2_eigen::tf2_eigen)
include(GenerateExportHeader)
generate_export_header(moveit_smoothing_base)
set_target_properties(moveit_smoothing_base
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

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
  moveit_acceleration_filter
  moveit_acceleration_filter_parameters
  moveit_robot_state
  moveit_smoothing_base
  osqp::osqp
  srdfdom::srdfdom
  pluginlib::pluginlib)
# OSQP shipped v1.0 in 2024 with a redesigned C API. Drive the source-code guard
# from the include directory the osqp::osqp target actually resolved to, so that
# a build environment with BOTH v0.6 (apt) and v1.0 (source-overlay) installed
# sees a self-consistent compile+link (the header-probe fallback in the .hpp can
# race the -I search order between the two installs). PUBLIC so consumers of
# moveit_acceleration_filter (notably test_acceleration_filter) see the same
# value — the guard controls a private member type of AccelerationLimitedPlugin,
# so an ABI mismatch between the .so and its test would be UB.
#
# Not using osqp_VERSION here: on v1.0.0 the shipped `osqp-config-version.cmake`
# sets `PACKAGE_VERSION "0.0.0"` (packaging bug), and v0.6.2 ships no
# config-version file at all — so osqp_VERSION is unusable across both streams,
# but for different reasons. Instead probe for a v1-only header inside the
# include dir that osqp::osqp exposes. See #3786.
get_target_property(_moveit_osqp_include_dirs osqp::osqp
                    INTERFACE_INCLUDE_DIRECTORIES)
set(_moveit_osqp_v1 0)
foreach(_dir IN LISTS _moveit_osqp_include_dirs)
  string(REGEX REPLACE "^\\$<BUILD_INTERFACE:(.*)>$" "\\1" _dir "${_dir}")
  string(REGEX REPLACE "^\\$<INSTALL_INTERFACE:(.*)>$" "\\1" _dir "${_dir}")
  if(EXISTS "${_dir}/osqp_api_types.h")
    set(_moveit_osqp_v1 1)
    break()
  endif()
endforeach()
target_compile_definitions(moveit_acceleration_filter
                           PUBLIC MOVEIT_OSQP_V1=${_moveit_osqp_v1})
unset(_moveit_osqp_v1)
unset(_moveit_osqp_include_dirs)

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
  moveit_robot_model moveit_smoothing_base srdfdom::srdfdom
  pluginlib::pluginlib)

add_library(moveit_ruckig_filter SHARED src/ruckig_filter.cpp)
generate_export_header(moveit_ruckig_filter)
target_include_directories(
  moveit_ruckig_filter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_ruckig_filter
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
generate_parameter_library(moveit_ruckig_filter_parameters
                           src/ruckig_filter_parameters.yaml)
target_link_libraries(
  moveit_ruckig_filter
  moveit_robot_state
  moveit_ruckig_filter_parameters
  moveit_smoothing_base
  ruckig::ruckig
  srdfdom::srdfdom
  pluginlib::pluginlib)

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
