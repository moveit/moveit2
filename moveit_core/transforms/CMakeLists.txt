set(MOVEIT_LIB_NAME moveit_transforms)

add_library(${MOVEIT_LIB_NAME} SHARED src/transforms.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${MOVEIT_LIB_NAME}
  PRIVATE "MOVEIT_CORE_BUILDING_LIBRARY")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  geometric_shapes
  tf2_eigen
  rclcpp
  rmw_implementation
  urdfdom
  urdfdom_headers
  Boost
)

install(DIRECTORY include/ DESTINATION include)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
endif()
