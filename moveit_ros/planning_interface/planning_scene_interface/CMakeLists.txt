set(MOVEIT_LIB_NAME moveit_planning_scene_interface)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_scene_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} moveit_msgs moveit_core moveit_ros_move_group)

set(MOVEIT_PYTHON_LIB_NAME planning_scene_interface)
pybind11_add_module(${MOVEIT_PYTHON_LIB_NAME} SHARED src/python_planning_scene_interface.cpp)
set_target_properties(${MOVEIT_PYTHON_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_PYTHON_LIB_NAME} PUBLIC ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${LIBS})

if(WIN32)
 set_target_properties(${MOVEIT_PYTHON_LIB_NAME} PROPERTIES SUFFIX .pyd)
endif()

install(DIRECTORY include/ DESTINATION include)
configure_build_install_location(${MOVEIT_PYTHON_LIB_NAME})