set(MOVEIT_LIB_NAME moveit_planning_scene)

if(WIN32)
  # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
else()
  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection")
endif()

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_scene.cpp)
#TODO: Fix the versioning
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
  ${octomap_msgs_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
  moveit_exceptions
  moveit_transforms
  moveit_collision_detection_fcl
  moveit_collision_detection
  moveit_kinematic_constraints
  moveit_robot_trajectory
  moveit_trajectory_processing
  ${LIBOCTOMAP_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${Boost_LIBRARIES}
  ${octomap_msgs_LIBRARIES}
	${OCTOMAP_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  ament_add_gtest(test_planning_scene test/test_planning_scene.cpp
		APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_planning_scene
		${MOVEIT_LIB_NAME}
		${srdfdom_LIBRARIES}
		${urdf_LIBRARIES}
		${geometric_shapes_LIBRARIES}
		${OCTOMAP_LIBRARIES}
	)
endif()
