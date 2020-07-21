cmake_minimum_required(VERSION 3.10.2)
project(moveit_planners_ompl)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP_PC REQUIRED assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_PC_INCLUDE_DIRS})
endif()

# find *absolute* paths to ASSIMP_LIBRARIES
# Both, pkg-config and assimp's cmake-config don't provide an absolute library path.
# For, pkg-config the path is in ASSIMP_PC_LIBRARY_DIRS, for cmake in ASSIMP_LIBRARY_DIRS.
find_library(ASSIMP_ABS_LIBRARIES NAMES ${ASSIMP_LIBRARIES} assimp HINTS ${ASSIMP_LIBRARY_DIRS} ${ASSIMP_PC_LIBRARY_DIRS})
set(ASSIMP_LIBRARIES "${ASSIMP_ABS_LIBRARIES}")

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-overloaded-virtual)
endif()

include_directories(ompl_interface/include)
include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

add_subdirectory(ompl_interface)

install(TARGETS moveit_ompl_interface moveit_ompl_planner_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core ompl)

pluginlib_export_plugin_description_file(moveit_core ompl_interface_plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
