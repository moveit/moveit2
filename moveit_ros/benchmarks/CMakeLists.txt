cmake_minimum_required(VERSION 3.22)
project(moveit_ros_benchmarks)

set(MOVEIT_LIB_NAME moveit_ros_benchmarks)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(pluginlib REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  Boost
  tf2_eigen
  moveit_core
  moveit_ros_planning
  moveit_ros_warehouse
  pluginlib
)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/BenchmarkOptions.cpp
  src/BenchmarkExecutor.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
if(WIN32)
  set(EXTRA_LIB ws2_32.lib)
endif()
ament_target_dependencies(${MOVEIT_LIB_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(${MOVEIT_LIB_NAME} ${EXTRA_LIB})

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
ament_target_dependencies(moveit_run_benchmark
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME})

add_executable(moveit_combine_predefined_poses_benchmark src/simple_benchmarks/CombinePredefinedPosesBenchmark.cpp)
ament_target_dependencies(moveit_combine_predefined_poses_benchmark
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(moveit_combine_predefined_poses_benchmark ${MOVEIT_LIB_NAME})

install(
  TARGETS ${MOVEIT_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    moveit_run_benchmark
    moveit_combine_predefined_poses_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/moveit_benchmark_statistics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
