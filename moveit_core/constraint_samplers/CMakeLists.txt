set(MOVEIT_LIB_NAME moveit_constraint_samplers)

add_library(${MOVEIT_LIB_NAME}
  src/constraint_sampler.cpp
  src/constraint_sampler_manager.cpp
  src/constraint_sampler_tools.cpp
  src/default_constraint_samplers.cpp
  src/union_constraint_sampler.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# target_link_libraries(${MOVEIT_LIB_NAME}
#   moveit_robot_state
#   moveit_kinematic_constraints
#   moveit_kinematics_base
#   moveit_planning_scene
#   ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_robot_state
  moveit_kinematic_constraints
  moveit_kinematics_base
  moveit_planning_scene
  "rclcpp"
  "rmw_implementation"
  "urdfdom"
  "urdfdom_headers"
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(orocos_kdl REQUIRED)
  find_package(angles REQUIRED)
  find_package(tf2_kdl REQUIRED)
  find_package(moveit_resources REQUIRED)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${angles_INCLUDE_DIRS} ${tf2_kdl_INCLUDE_DIRS} ${moveit_resources_INCLUDE_DIRS})

  ament_add_gtest(test_constraint_samplers
    test/test_constraint_samplers.cpp
    test/pr2_arm_kinematics_plugin.cpp
    test/pr2_arm_ik.cpp
  )
  # target_link_libraries(test_constraint_samplers
  #   moveit_test_utils
  #   ${MOVEIT_LIB_NAME}
  #   ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES}
  #   ${angles_LIBRARIES}
  #   ${orocos_kdl_LIBRARIES}
  #   ${urdfdom_LIBRARIES}
  #   ${urdfdom_headers_LIBRARIES}
  # )
  ament_target_dependencies(test_constraint_samplers
    moveit_test_utils
    ${MOVEIT_LIB_NAME}
    "rclcpp"
    "rmw_implementation"
    "angles"
    "orcos_kdl"
    "urdfdom"
    "urdfdom_headers"
  )

endif()
