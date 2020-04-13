set(MOVEIT_LIB_NAME moveit_planning_scene_interface)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_scene_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#target_link_libraries(${MOVEIT_LIB_NAME} moveit_common_planning_interface_objects ${LIBS} ${Boost_LIBRARIES})
#ament_target_dependencies(${MOVEIT_LIB_NAME} )


#add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_planning_scene_interface.cpp)
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES} moveit_py_bindings_tools)
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_planning_scene_interface PREFIX "")
#set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
if(WIN32)
#  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif(WIN32)

install(TARGETS ${MOVEIT_LIB_NAME}
  EXPORT ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#install(TARGETS ${MOVEIT_LIB_NAME}_python
#  DESTINATION bin)

install(DIRECTORY include/ DESTINATION planning_scene_interface/include)

#for <moveit/move_group/capability_names.h>
find_package(moveit_ros_move_group)
include_directories(${moveit_ros_move_group_INCLUDE_DIRS})
#ament_export_include_directories(${moveit_ros_move_group_INCLUDE_DIRS})

ament_export_dependencies(${MOVEIT_LIB_NAME})
#ament_export_interfaces(${MOVEIT_LIB_NAME} HAS_LIBRARY_TARGET)
