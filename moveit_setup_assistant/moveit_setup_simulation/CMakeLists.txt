cmake_minimum_required(VERSION 3.10.2)
project(moveit_setup_simulation)

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
   include/moveit_setup_simulation/simulation_widget.hpp
)

add_library(${PROJECT_NAME}
    src/simulation.cpp
    src/simulation_widget.cpp
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
endif()

install(DIRECTORY templates DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(moveit_setup_framework moveit_setup_framework_plugins.xml)

ament_package()
