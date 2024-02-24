cmake_minimum_required(VERSION 3.22)
project(moveit_common NONE)

find_package(ament_cmake REQUIRED)

ament_package(CONFIG_EXTRAS "moveit_common-extras.cmake")

install(DIRECTORY cmake DESTINATION share/moveit_common)
