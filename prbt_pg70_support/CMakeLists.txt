cmake_minimum_required(VERSION 2.8.3)
project(moveit_resources_prbt_pg70_support)

find_package(catkin REQUIRED)
catkin_package()

#############
## Install ##
#############
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
