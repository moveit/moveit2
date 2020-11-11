cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_benchmarks)

set(MOVEIT_LIB_NAME moveit_ros_benchmarks)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED date_time filesystem)
find_package(tf2_eigen REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/BenchmarkOptions.cpp
  src/BenchmarkExecutor.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  Boost
  tf2_eigen
  moveit_ros_planning
  moveit_ros_warehouse
  pluginlib
)

add_executable(moveit_run_benchmark src/RunBenchmark.cpp)
ament_target_dependencies(moveit_run_benchmark
  rclcpp
  Boost
  tf2_eigen
  moveit_ros_planning
  moveit_ros_warehouse
  pluginlib
)
target_link_libraries(moveit_run_benchmark ${MOVEIT_LIB_NAME})

add_executable(moveit_combine_predefined_poses_benchmark src/simple_benchmarks/CombinePredefinedPosesBenchmark.cpp)
ament_target_dependencies(moveit_combine_predefined_poses_benchmark
  rclcpp
  Boost
  tf2_eigen
  moveit_ros_planning
  moveit_ros_warehouse
  pluginlib
)
target_link_libraries(moveit_combine_predefined_poses_benchmark ${MOVEIT_LIB_NAME})

ament_export_include_directories(include)
ament_export_libraries(${MOVEIT_LIB_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(Boost)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_warehouse)
ament_export_dependencies(pluginlib)

install(
  TARGETS ${MOVEIT_LIB_NAME}
  EXPORT  ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_package()
