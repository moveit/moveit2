add_library(moveit_rdf_loader SHARED src/rdf_loader.cpp
                                     src/synchronized_string_parameter.cpp)
set_target_properties(moveit_rdf_loader PROPERTIES VERSION
                                                   "${${PROJECT_NAME}_VERSION}")
      target_link_libraries(moveit_rdf_loader PUBLIC
        ament_index_cpp::ament_index_cpp
        moveit_core::moveit_collision_detection
        moveit_core::moveit_collision_detection_bullet
        moveit_core::moveit_collision_detection_fcl
        moveit_core::moveit_collision_distance_field
        moveit_core::moveit_constraint_samplers
        moveit_core::moveit_distance_field
        moveit_core::moveit_dynamics_solver
        moveit_core::moveit_exceptions
        moveit_core::moveit_kinematic_constraints
        moveit_core::moveit_kinematics_base
        moveit_core::moveit_kinematics_metrics
        moveit_core::moveit_macros
        moveit_core::moveit_planning_interface
        moveit_core::moveit_planning_scene
        moveit_core::moveit_robot_model
        moveit_core::moveit_robot_state
        moveit_core::moveit_robot_trajectory
        moveit_core::moveit_smoothing_base
        moveit_core::moveit_test_utils
        moveit_core::moveit_trajectory_processing
        moveit_core::moveit_transforms
        moveit_core::moveit_utils
        rclcpp::rclcpp
        srdfdom::srdfdom
        urdf::urdf
      )

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(DIRECTORY test/data test/launch
        DESTINATION share/${PROJECT_NAME}/rdf_loader/test)

# TODO(sjahr) Debug and re-enable if(BUILD_TESTING)
# find_package(ament_cmake_gtest REQUIRED) find_package(ros_testing REQUIRED)
#
# ament_add_gtest_executable(test_rdf_integration test/test_rdf_integration.cpp
# ) target_link_libraries(test_rdf_integration moveit_rdf_loader)
# add_ros_test(test/launch/test_rdf_integration.test.py TIMEOUT 120)
#
# add_executable(boring_string_publisher test/boring_string_publisher.cpp)
# target_link_libraries(boring_string_publisher moveit_rdf_loader)
#
# install( TARGETS test_rdf_integration boring_string_publisher ARCHIVE
# DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION
# lib/${PROJECT_NAME} ) endif()
