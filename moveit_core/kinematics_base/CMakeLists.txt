cmake_minimum_required(VERSION 3.5)
set(MOVEIT_LIB_NAME moveit_kinematics_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${MOVEIT_LIB_NAME} src/kinematics_base.cpp)

# set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

# Avoid warnings about deprecated members of KinematicsBase when building KinematicsBase itself.
# TODO: remove when deperecated variables (tip_frame_, search_discretization_) are finally removed.
# target_compile_options(${MOVEIT_LIB_NAME} PRIVATE -Wno-deprecated-declarations)

# This line is needed to ensure that messages are done being built before this is built
# add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${MOVEIT_LIB_NAME} ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES} ${common_interfaces_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom_headers)

# ament_target_dependencies(${MOVEIT_LIB_NAME}
# )

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/
				DESTINATION include)
