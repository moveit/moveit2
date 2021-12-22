cmake_minimum_required(VERSION 3.10.2)
project(moveit_resources_prbt_support)

find_package(ament_cmake REQUIRED)
ament_package()

#############
## Install ##
#############

## Install URDF and meshes
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
