cmake_minimum_required(VERSION 3.10.2)
project(moveit_kinematics)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED program_options system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(random_numbers REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
    kdl_kinematics_plugin/include
    lma_kinematics_plugin/include
    srv_kinematics_plugin/include
    cached_ik_kinematics_plugin/include
)

pluginlib_export_plugin_description_file(moveit_core kdl_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core lma_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core srv_kinematics_plugin_description.xml)
pluginlib_export_plugin_description_file(moveit_core cached_ik_kinematics_plugin_description.xml)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(lma_kinematics_plugin)
add_subdirectory(srv_kinematics_plugin)
add_subdirectory(ikfast_kinematics_plugin)
add_subdirectory(cached_ik_kinematics_plugin)
add_subdirectory(test)

install(TARGETS
  moveit_kdl_kinematics_plugin
  moveit_lma_kinematics_plugin
  moveit_srv_kinematics_plugin
  moveit_cached_ik_kinematics_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
    moveit_kdl_kinematics_plugin
    moveit_lma_kinematics_plugin
    moveit_srv_kinematics_plugin
    moveit_cached_ik_kinematics_plugin)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(Boost)
ament_package()

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(launch_testing_ament_cmake REQUIRED)
#   find_package(moveit_resources REQUIRED)
#   find_package(moveit_ros_planning REQUIRED)

#   ament_add_gtest_executable(test_kinematics_plugin test/test_kinematics_plugin.cpp)
#   ament_target_dependencies(test_kinematics_plugin moveit_ros_planning pluginlib)
#   # target_link_libraries(test_kinematics_plugin)

#   if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
#     target_compile_options(test_kinematics_plugin PRIVATE -Wno-deprecated-declarations)
#   endif()

#   set(DEPS DEPENDENCIES test_kinematics_plugin)
#   set(ARGS ARGS ik_plugin:=kdl_kinematics_plugin/KDLKinematicsPlugin)
#   add_launch_test(test/launch/fanuc-kdl-singular.test.py)
#   add_launch_test(test/launch/fanuc-kdl.test.py)
#   add_launch_test(test/launch/panda-kdl-singular.test.py)
#   add_launch_test(test/launch/panda-kdl.test.py)
#   # add_launch_test(panda-kdl.test ${DEPS} ${ARGS})

#   # LMA solver seems to have less precision
#   #set(ARGS ARGS ik_plugin:=lma_kinematics_plugin/LMAKinematicsPlugin tolerance:=0.0005)
#   # add_launch_test(fanuc-kdl-test.py)# ${DEPS} ${ARGS})
#   # add_launch_test(panda-kdl.test ${DEPS} ${ARGS})

#   # Run ikfast tests only if the corresponding packages were built
#   find_package(fanuc_ikfast_plugin QUIET)
#   if(fanuc_ikfast_plugin_FOUND)
#     add_launch_test(test/launch/fanuc-ikfast.test.py)
#     add_launch_test(test/launch/fanuc-ikfast-singular.test.py)
#   endif()

#   find_package(panda_ikfast_plugin QUIET)
#   if(panda_ikfast_plugin_FOUND)
#     add_launch_test(test/launch/panda-ikfast.test.py)
#     add_launch_test(test/launch/panda-ikfast-singular.test.py)
#   endif()

#   # Benchmarking program for cached_ik_kinematics
#   add_executable(benchmark_ik test/benchmark_ik.cpp)
#   ament_target_dependencies(
#     benchmark_ik
#     rclcpp
#     moveit_core
#     moveit_ros_planning
#     Boost
#   )

#   install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME})
#   install(DIRECTORY test/launch DESTINATION share/${PROJECT_NAME})

#   install(TARGETS benchmark_ik RUNTIME DESTINATION bin)
#   install(TARGETS test_kinematics_plugin DESTINATION lib/${PROJECT_NAME})
# endif()
