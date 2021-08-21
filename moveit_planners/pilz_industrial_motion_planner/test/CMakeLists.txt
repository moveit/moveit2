find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros_testing REQUIRED)
find_package(moveit_resources_prbt_support REQUIRED)
find_package(moveit_resources_prbt_moveit_config REQUIRED)
find_package(Boost REQUIRED thread)

find_package(${PROJECT_NAME}_testutils REQUIRED)

# Add test directory as include directory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# pilz_industrial_motion_testhelpers
add_library(${PROJECT_NAME}_testhelpers
  test_utils.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_generator.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_generator_circ.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_generator_lin.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_generator_ptp.cpp
  ${CMAKE_SOURCE_DIR}/src/path_circle_generator.cpp
  ${CMAKE_SOURCE_DIR}/src/velocity_profile_atrap.cpp
)
ament_target_dependencies(${PROJECT_NAME}_testhelpers ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}_testhelpers ${PROJECT_NAME}_lib)

# Unit tests
add_subdirectory(unit_tests)

# Integration tests
add_subdirectory(integration_tests)
# Install test data
install(DIRECTORY test_data DESTINATION share/${PROJECT_NAME})
