find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros_testing REQUIRED)
find_package(moveit_resources_prbt_support REQUIRED)
find_package(moveit_resources_prbt_moveit_config REQUIRED)
find_package(Boost REQUIRED thread)

find_package(${PROJECT_NAME}_testutils REQUIRED)

# Add test directory as include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# pilz_industrial_motion_testhelpers
add_library(${PROJECT_NAME}_test_utils test_utils.cpp)
ament_target_dependencies(${PROJECT_NAME}_test_utils
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}_test_utils joint_limits_common
                      trajectory_generation_common)

# Unit tests
add_subdirectory(unit_tests)

# Integration tests
add_subdirectory(integration_tests)
# Install test data
install(DIRECTORY test_data DESTINATION share/${PROJECT_NAME})
