set(MOVEIT_LIB_NAME moveit_robot_interface)

include_directories(${PYTHON_INCLUDE_DIRS})

add_library(${MOVEIT_LIB_NAME} SHARED src/robot_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_common_planning_interface_objects)

set(MOVEIT_PYTHON_LIB_NAME robot_interface)
pybind11_add_module(${MOVEIT_PYTHON_LIB_NAME} SHARED src/python_robot_interface.cpp)
set_target_properties(${MOVEIT_PYTHON_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_PYTHON_LIB_NAME} PUBLIC ${MOVEIT_LIB_NAME} moveit_common_planning_interface_objects ${PYTHON_LIBRARIES})

if(WIN32)
  set_target_properties(${MOVEIT_PYTHON_LIB_NAME} PROPERTIES SUFFIX .pyd)
endif()

install(DIRECTORY include/ DESTINATION include)
configure_build_install_location(${MOVEIT_PYTHON_LIB_NAME})
