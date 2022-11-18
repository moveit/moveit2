cmake_minimum_required(VERSION 3.22)
project(moveit_setup_controllers)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
    include/moveit_setup_controllers/controller_edit_widget.hpp
    include/moveit_setup_controllers/controllers_widget.hpp
    include/moveit_setup_controllers/urdf_modifications_widget.hpp
)

add_library(${PROJECT_NAME}
    src/control_xacro_config.cpp
    src/controller_edit_widget.cpp
    src/controllers.cpp
    src/controllers_widget.cpp
    src/controllers_config.cpp
    src/modified_urdf_config.cpp
    src/moveit_controllers_config.cpp
    src/moveit_controllers_widget.cpp
    src/ros2_controllers_config.cpp
    src/ros2_controllers_widget.cpp
    src/urdf_modifications.cpp
    src/urdf_modifications_widget.cpp
    ${MOC_FILES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  moveit_setup_framework
  pluginlib
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers ${PROJECT_NAME})
endif()

install(DIRECTORY templates DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(moveit_setup_framework moveit_setup_framework_plugins.xml)

ament_package()
