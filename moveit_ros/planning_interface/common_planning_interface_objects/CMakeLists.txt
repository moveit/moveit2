set(MOVEIT_LIB_NAME moveit_common_planning_interface_objects)

if (WIN32)
  add_compile_options(/wd4251 /wd4068 /wd4275)
endif()

find_package(Boost REQUIRED COMPONENTS thread)

add_library(${MOVEIT_LIB_NAME} SHARED src/common_objects.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

ament_target_dependencies(${MOVEIT_LIB_NAME}
    Boost
    rclcpp
    moveit_ros_planning
    tf2_ros
)

install(DIRECTORY include/ DESTINATION include)
