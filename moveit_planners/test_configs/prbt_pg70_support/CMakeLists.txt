cmake_minimum_required(VERSION 3.22)
project(moveit_resources_prbt_pg70_support)

find_package(ament_cmake REQUIRED)
ament_package()

# ##############################################################################
# Install ##
# ##############################################################################
install(DIRECTORY config DESTINATION share/moveit_resources_prbt_pg70_support)
install(DIRECTORY meshes DESTINATION share/moveit_resources_prbt_pg70_support)
install(DIRECTORY urdf DESTINATION share/moveit_resources_prbt_pg70_support)
