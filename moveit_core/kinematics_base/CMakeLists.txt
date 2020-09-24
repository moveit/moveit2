cmake_minimum_required(VERSION 3.5)
set(MOVEIT_LIB_NAME moveit_kinematics_base)

add_library(${MOVEIT_LIB_NAME} SHARED src/kinematics_base.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# This line is needed to ensure that messages are done being built before this is built
ament_target_dependencies(${MOVEIT_LIB_NAME} rclcpp urdf urdfdom_headers moveit_msgs geometry_msgs Boost)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE "KINEMATICS_BASE_BUILDING_LIBRARY")

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/
        DESTINATION include)
