if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(moveit_resources REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

  ament_add_gtest_executable(test_kinematics_plugin test_kinematics_plugin.cpp)
  ament_target_dependencies(test_kinematics_plugin moveit_ros_planning)
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(test_kinematics_plugin PRIVATE -Wno-deprecated-declarations)
  endif()

  set(DEPS DEPENDENCIES test_kinematics_plugin)
  set(ARGS ARGS ik_plugin:=kdl_kinematics_plugin/KDLKinematicsPlugin)
  add_launch_test(fanuc-kdl.test.py)# ${DEPS} ${ARGS})
  # add_launch_test(panda-kdl.test ${DEPS} ${ARGS})

  # LMA solver seems to have less precision
  set(ARGS ARGS ik_plugin:=lma_kinematics_plugin/LMAKinematicsPlugin tolerance:=0.0005)
  # add_launch_test(fanuc-kdl-test.py)# ${DEPS} ${ARGS})
  # add_launch_test(panda-kdl.test ${DEPS} ${ARGS})

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

  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

  install(TARGETS benchmark_ik RUNTIME DESTINATION bin)
  install(TARGETS test_kinematics_plugin DESTINATION lib/${PROJECT_NAME})
endif()
