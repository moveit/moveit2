set(MOVEIT_LIB_NAME moveit_cpp)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/moveit_cpp.cpp
  src/planning_component.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  Boost
  rclcpp
  moveit_core
  moveit_ros_planning
)

install(DIRECTORY include/ DESTINATION include)
