set(MOVEIT_LIB_NAME moveit_move_group_interface)

add_library(${MOVEIT_LIB_NAME} SHARED src/move_group_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_common_planning_interface_objects moveit_planning_scene_interface ${Boost_THREAD_LIBRARY})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_ros_occupancy_map_monitor
  moveit_ros_move_group
  moveit_ros_planning
  moveit_msgs
  moveit_core
)

# TODO (ddengster) : port wrap_python_move_group
#add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_move_group.cpp)
#target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${eigenpy_LIBRARIES} ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES} moveit_py_bindings_tools)
#add_dependencies(${MOVEIT_LIB_NAME}_python)
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_move_group_interface PREFIX "")
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
if(WIN32)
#  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif()

install(TARGETS ${MOVEIT_LIB_NAME}
  EXPORT ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install(TARGETS ${MOVEIT_LIB_NAME}_python
#  ARCHIVE DESTINATION bin
#  LIBRARY DESTINATION bin
#  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

#add_executable(demo src/demo.cpp)
#target_link_libraries(demo ${MOVEIT_LIB_NAME} ${LIBS} ${Boost_LIBRARIES})
