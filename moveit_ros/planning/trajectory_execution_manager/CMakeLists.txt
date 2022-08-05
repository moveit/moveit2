set(MOVEIT_LIB_NAME moveit_trajectory_execution_manager)

add_library(${MOVEIT_LIB_NAME} SHARED src/trajectory_execution_manager.cpp)
include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_core
  moveit_ros_occupancy_map_monitor
  rclcpp
  pluginlib
  std_msgs
  sensor_msgs
  moveit_msgs
  tf2_eigen
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_planning_scene_monitor
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MOVEIT_LIB_NAME}_export.h DESTINATION include)

if(CATKIN_ENABLE_TESTING)
## This needs further cleanup before it can run
# add_library(test_controller_manager_plugin test/test_moveit_controller_manager_plugin.cpp)
# set_target_properties(test_controller_manager_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# target_link_libraries(test_controller_manager_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
# find_package(moveit_resources_panda_moveit_config REQUIRED)
# find_package(rostest REQUIRED)
# add_rostest_gtest(test_execution_manager
#                   test/test_execution_manager.test
#                   test/test_execution_manager.cpp)
# target_link_libraries(test_execution_manager ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
