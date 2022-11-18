cmake_minimum_required(VERSION 3.22)
project(moveit_resources_prbt_pg70_support)

find_package(ament_cmake REQUIRED)
ament_package()

#############
## Install ##
#############
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
