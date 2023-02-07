add_library(moveit_py_bindings_tools src/roscpp_initializer.cpp)
set_target_properties(moveit_py_bindings_tools PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_py_bindings_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

install(TARGETS moveit_py_bindings_tools
  EXPORT moveit_py_bindings_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

add_library(moveit_py_bindings_tools_python src/wrap_python_roscpp_initializer.cpp)
target_link_libraries(moveit_py_bindings_tools_python moveit_py_bindings_tools ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(moveit_py_bindings_tools_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
set_target_properties(moveit_py_bindings_tools_python PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
set_target_properties(moveit_py_bindings_tools_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}")
if(WIN32)
  set_target_properties(moveit_py_bindings_tools_python PROPERTIES SUFFIX .pyd)
endif()

install(TARGETS moveit_py_bindings_tools_python
  EXPORT moveit_py_bindings_tools_python
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
