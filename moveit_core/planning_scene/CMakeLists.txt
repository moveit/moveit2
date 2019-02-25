set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} src/planning_scene.cpp)
#TODO: Fix the versioning
# set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
  moveit_exceptions
  moveit_transforms
  moveit_collision_detection_fcl
  moveit_kinematic_constraints
  moveit_robot_trajectory
  moveit_trajectory_processing
  ${LIBOCTOMAP_LIBRARIES} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

#TODO: Fix this so it can be built with colcon
# if(BUILD_TESTING)
#   find_package(moveit_resources REQUIRED)
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#
#   ament_add_gtest(test_planning_scene test/test_planning_scene.cpp)
#   target_link_libraries(test_planning_scene ${MOVEIT_LIB_NAME})
# endif()
