if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

# TODO(henningkayser): enable new tests
#  ament_add_gtest(test_kinematics_plugin test_kinematics_plugin.cpp APPEND_LIBRARY_DIRS "${append_library_dirs}")
#
#  ament_target_dependencies(
#    test_kinematics_plugin
#    class_loader
#    pluginlib
#    rclcpp
#    moveit_core
#    moveit_ros_planning
#    Boost
#  )
#  target_compile_definitions(test_kinematics_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
#
# catkin_add_executable_with_gtest(test_kinematics_plugin test_kinematics_plugin.cpp)
#  target_link_libraries(test_kinematics_plugin ${catkin_LIBRARIES} ${moveit_ros_planning_LIBRARIES} ${xmlrpcpp_LIBRARIES})
#  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
#    target_compile_options(test_kinematics_plugin PRIVATE -Wno-deprecated-declarations)
#  endif()
#
#  set(DEPS DEPENDENCIES test_kinematics_plugin)
#  set(ARGS ARGS ik_plugin:=kdl_kinematics_plugin/KDLKinematicsPlugin)
#  add_rostest(fanuc-kdl.test ${DEPS} ${ARGS})
#  add_rostest(panda-kdl.test ${DEPS} ${ARGS})
#
#  # LMA solver seems to have less precision
#  set(ARGS ARGS ik_plugin:=lma_kinematics_plugin/LMAKinematicsPlugin tolerance:=0.0005)
#  add_rostest(fanuc-kdl.test ${DEPS} ${ARGS})
#  add_rostest(panda-kdl.test ${DEPS} ${ARGS})

  # Run ikfast tests only if the corresponding packages were built
  find_package(fanuc_ikfast_plugin QUIET)
  if(fanuc_ikfast_plugin_FOUND)
    add_rostest(fanuc-ikfast.test ${DEPS})
  endif()

  find_package(panda_ikfast_plugin QUIET)
  if(panda_ikfast_plugin_FOUND)
    add_rostest(panda-ikfast.test ${DEPS})
  endif()

  # Benchmarking program for cached_ik_kinematics
  add_executable(benchmark_ik benchmark_ik.cpp)
  ament_target_dependencies(
    benchmark_ik
    rclcpp
    moveit_core
    moveit_ros_planning
    Boost
  )

  install(TARGETS benchmark_ik DESTINATION lib/${PROJECT_NAME})
endif()
