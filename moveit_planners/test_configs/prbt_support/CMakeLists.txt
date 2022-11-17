cmake_minimum_required(VERSION 3.22)
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
