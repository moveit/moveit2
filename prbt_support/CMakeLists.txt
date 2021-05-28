cmake_minimum_required(VERSION 2.8.3)
project(moveit_resources_prbt_support)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############

## Install URDF and meshes
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
