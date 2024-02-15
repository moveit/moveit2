cmake_minimum_required(VERSION 3.22)
project(moveit_resources_prbt_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/moveit_resources_prbt_moveit_config
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/moveit_resources_prbt_moveit_config)

ament_package()
