cmake_minimum_required(VERSION 3.10.2)
project(moveit_common NONE)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "moveit_common-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
