if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(moveit_resources_fanuc_description REQUIRED)
  find_package(moveit_resources_fanuc_moveit_config REQUIRED)
  find_package(moveit_resources_panda_description REQUIRED)
  find_package(moveit_resources_panda_moveit_config REQUIRED)
  find_package(moveit_ros_planning REQUIRED)

  ament_add_gtest_executable(test_kinematics_plugin test_kinematics_plugin.cpp)
  ament_target_dependencies(test_kinematics_plugin moveit_ros_planning pluginlib)
  target_link_libraries(test_kinematics_plugin moveit_kdl_kinematics_plugin)

  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(test_kinematics_plugin PRIVATE -Wno-deprecated-declarations)
  endif()

  # KDL testing
  set(ARGS ARGS ik_plugin:=kdl_kinematics_plugin/KDLKinematicsPlugin)
  add_ros_test(launch/fanuc-kdl-singular.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(launch/fanuc-kdl.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(launch/panda-kdl-singular.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(launch/panda-kdl.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # LMA testing
  set(ARGS ARGS ik_plugin:=lma_kinematics_plugin/LMAKinematicsPlugin)
  add_ros_test(launch/fanuc-lma-singular.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(launch/fanuc-lma.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(launch/panda-lma-singular.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(launch/panda-lma.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Run ikfast tests only if the corresponding packages were built
  # TODO (vatanaksoytezer): Enable ikfast tests
  # find_package(fanuc_ikfast_plugin QUIET)
  # if(fanuc_ikfast_plugin_FOUND)
  #   add_ros_test(launch/fanuc-ikfast.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  #   add_ros_test(launch/fanuc-ikfast-singular.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # endif()

  # find_package(panda_ikfast_plugin QUIET)
  # if(panda_ikfast_plugin_FOUND)
  #   add_ros_test(launch/panda-ikfast.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  #   add_ros_test(launch/panda-ikfast-singular.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  # endif()

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
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

  install(TARGETS benchmark_ik RUNTIME DESTINATION bin)
  install(TARGETS test_kinematics_plugin DESTINATION lib/${PROJECT_NAME})
endif()
