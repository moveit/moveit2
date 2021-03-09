set(MOVEIT_LIB_NAME moveit_planning_scene_interface)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_scene_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} moveit_msgs moveit_core moveit_ros_move_group)

# TODO(JafarAbdi): Support python wrapper
#add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_planning_scene_interface.cpp)
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES} moveit_py_bindings_tools)
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_planning_scene_interface PREFIX "")
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
if(WIN32)
#  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif()

#install(TARGETS ${MOVEIT_LIB_NAME}_python
#  EXPORT ${MOVEIT_LIB_NAME}_python
#  DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
