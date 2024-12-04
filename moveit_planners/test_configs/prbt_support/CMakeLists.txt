cmake_minimum_required(VERSION 3.22)
project(moveit_resources_prbt_support)

find_package(ament_cmake REQUIRED)
ament_package()

# ##############################################################################
# Install ##
# ##############################################################################

# Install URDF and meshes
install(DIRECTORY urdf DESTINATION share/moveit_resources_prbt_support)
install(DIRECTORY meshes DESTINATION share/moveit_resources_prbt_support)
install(DIRECTORY config DESTINATION share/moveit_resources_prbt_support)
