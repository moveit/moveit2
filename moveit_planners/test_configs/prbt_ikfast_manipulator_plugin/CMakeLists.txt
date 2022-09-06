cmake_minimum_required(VERSION 3.22)
project(moveit_resources_prbt_ikfast_manipulator_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT WIN32)
  add_compile_options(-Wall)
  add_compile_options(-Wextra)
  add_compile_options(-Wno-unused-parameter)
  add_compile_options(-Wno-unused-variable)
endif()

# enable aligned new in gcc7+
if(CMAKE_COMPILER_IS_GNUCXX)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -faligned-new")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(prbt_manipulator_moveit_ikfast_plugin SHARED
  src/prbt_manipulator_ikfast_moveit_plugin.cpp)

if(NOT WIN32)
  # suppress warnings about unused variables in OpenRave's solver code
  target_compile_options(prbt_manipulator_moveit_ikfast_plugin PRIVATE -Wno-unused-variable)
endif()
ament_target_dependencies(prbt_manipulator_moveit_ikfast_plugin
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  tf2_eigen
  tf2_eigen_kdl
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(moveit_core prbt_manipulator_moveit_ikfast_plugin_description.xml)

install(
  TARGETS prbt_manipulator_moveit_ikfast_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
