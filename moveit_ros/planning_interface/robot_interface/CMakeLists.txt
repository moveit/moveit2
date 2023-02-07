add_library(moveit_robot_interface_python src/wrap_python_robot_interface.cpp)
target_link_libraries(moveit_robot_interface_python ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  moveit_common_planning_interface_objects moveit_py_bindings_tools
)
set_target_properties(moveit_robot_interface_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(moveit_robot_interface_python PROPERTIES OUTPUT_NAME _moveit_robot_interface PREFIX "")
set_target_properties(moveit_robot_interface_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
if(WIN32)
  set_target_properties(moveit_robot_interface_python PROPERTIES SUFFIX .pyd)
endif()

install(TARGETS moveit_robot_interface_python
  EXPORT moveit_robot_interface_python
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
