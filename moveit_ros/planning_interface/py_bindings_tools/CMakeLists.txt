set(MOVEIT_LIB_NAME moveit_py_bindings_tools)

add_library(${MOVEIT_LIB_NAME} src/roscpp_initializer.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${LIBS} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(${MOVEIT_LIB_NAME}_python src/wrap_python_roscpp_initializer.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_python ${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
if(WIN32)
  set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
endif(WIN32)

install(TARGETS ${MOVEIT_LIB_NAME}_python
  DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

#for <moveit/move_group/capability_names.h>
find_package(moveit_ros_move_group)
include_directories(${moveit_ros_move_group_INCLUDE_DIRS})
ament_export_include_directories(${moveit_ros_move_group_INCLUDE_DIRS})
