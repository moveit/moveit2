cmake_minimum_required(VERSION 3.22)
project(moveit_ros_benchmarks LANGUAGES CXX)

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
    pluginlib)

add_library(moveit_ros_benchmarks SHARED src/BenchmarkOptions.cpp
                                         src/BenchmarkExecutor.cpp)
set_target_properties(moveit_ros_benchmarks
                      PROPERTIES VERSION "${moveit_ros_benchmarks_VERSION}")
if(WIN32)
  set(EXTRA_LIB ws2_32.lib)
endif()
ament_target_dependencies(moveit_ros_benchmarks ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_ros_benchmarks ${EXTRA_LIB})

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
ament_target_dependencies(moveit_run_benchmark ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_run_benchmark moveit_ros_benchmarks)

install(
  TARGETS moveit_ros_benchmarks
  EXPORT moveit_ros_benchmarksTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_benchmarks)

ament_export_targets(moveit_ros_benchmarksTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS moveit_run_benchmark DESTINATION lib/moveit_ros_benchmarks)

install(DIRECTORY include/ DESTINATION include/moveit_ros_benchmarks)

install(PROGRAMS scripts/moveit_benchmark_statistics.py
        DESTINATION lib/moveit_ros_benchmarks)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
