cmake_minimum_required(VERSION 3.16.3)
project(run_ompl_constrained_planning)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(run_ompl_constrained_planning src/run_ompl_constrained_planning.cpp)
add_executable(tool_path_demo src/tool_path_demo.cpp)

ament_target_dependencies(run_ompl_constrained_planning
  moveit_ros_planning_interface
  Boost
)

ament_target_dependencies(tool_path_demo
  moveit_ros_planning_interface
  Boost
)

install(
  TARGETS
    run_ompl_constrained_planning
    tool_path_demo
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})

ament_package()
