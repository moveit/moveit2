cmake_minimum_required(VERSION 3.8)
project(stomp_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(stomp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(stomp_moveit_example src/stomp_moveit_example.cpp)
target_include_directories(stomp_moveit_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stomp_moveit_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  stomp_moveit_example
  moveit_core
  moveit_ros
  moveit_visual_tools
)

target_link_libraries(stomp_moveit_example stomp::stomp)

install(TARGETS stomp_moveit_example
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
