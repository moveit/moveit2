cmake_minimum_required(VERSION 3.5)
set(MOVEIT_LIB_NAME moveit_kinematics_base)

add_library(${MOVEIT_LIB_NAME} SHARED src/kinematics_base.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# Avoid warnings about deprecated members of KinematicsBase when building KinematicsBase itself.
# TODO: remove when deperecated variables (tip_frame_, search_discretization_) are finally removed.
# target_compile_options(${MOVEIT_LIB_NAME} PRIVATE -Wno-deprecated-declarations)

# This line is needed to ensure that messages are done being built before this is built
ament_target_dependencies(${MOVEIT_LIB_NAME} urdf urdfdom_headers)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)
