set(MOVEIT_LIB_NAME moveit_warehouse)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/moveit_message_storage.cpp
  src/planning_scene_storage.cpp
  src/planning_scene_world_storage.cpp
  src/constraints_storage.cpp
  src/trajectory_constraints_storage.cpp
  src/state_storage.cpp
  src/warehouse_connector.cpp
)
include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} rclcpp Boost moveit_core warehouse_ros moveit_ros_planning)

add_executable(moveit_warehouse_broadcast src/broadcast.cpp)
ament_target_dependencies(moveit_warehouse_broadcast rclcpp Boost warehouse_ros moveit_ros_planning)
target_link_libraries(moveit_warehouse_broadcast ${MOVEIT_LIB_NAME})

add_executable(moveit_save_to_warehouse src/save_to_warehouse.cpp)
ament_target_dependencies(moveit_save_to_warehouse rclcpp Boost warehouse_ros moveit_ros_planning)
target_link_libraries(moveit_save_to_warehouse ${MOVEIT_LIB_NAME})

add_executable(moveit_warehouse_import_from_text src/import_from_text.cpp)
ament_target_dependencies(moveit_warehouse_import_from_text rclcpp Boost warehouse_ros moveit_ros_planning)
target_link_libraries(moveit_warehouse_import_from_text ${MOVEIT_LIB_NAME})

add_executable(moveit_warehouse_save_as_text src/save_as_text.cpp)
ament_target_dependencies(moveit_warehouse_save_as_text rclcpp Boost warehouse_ros moveit_ros_planning)
target_link_libraries(moveit_warehouse_save_as_text ${MOVEIT_LIB_NAME})

add_executable(moveit_init_demo_warehouse src/initialize_demo_db.cpp)
ament_target_dependencies(moveit_init_demo_warehouse rclcpp Boost warehouse_ros moveit_ros_planning)
target_link_libraries(moveit_init_demo_warehouse ${MOVEIT_LIB_NAME})

add_executable(moveit_warehouse_services src/warehouse_services.cpp)
ament_target_dependencies(moveit_warehouse_services rclcpp Boost warehouse_ros moveit_ros_planning)
target_link_libraries(moveit_warehouse_services ${MOVEIT_LIB_NAME})

install(
  TARGETS
    moveit_save_to_warehouse
    moveit_warehouse_broadcast
    moveit_warehouse_import_from_text
    moveit_warehouse_save_as_text
    moveit_init_demo_warehouse
    moveit_warehouse_services
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MOVEIT_LIB_NAME}_export.h DESTINATION include)
