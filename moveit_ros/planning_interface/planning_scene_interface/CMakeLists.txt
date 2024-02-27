add_library(moveit_planning_scene_interface SHARED
            src/planning_scene_interface.cpp)
set_target_properties(moveit_planning_scene_interface
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_planning_scene_interface moveit_msgs
                          moveit_core moveit_ros_move_group)

# TODO(JafarAbdi): Support python wrapper
# add_library(moveit_planning_scene_interface_python
# src/wrap_python_planning_scene_interface.cpp)
# set_target_properties(moveit_planning_scene_interface_python PROPERTIES
# VERSION "${${PROJECT_NAME}_VERSION}")
# target_link_libraries(moveit_planning_scene_interface_python
# moveit_planning_scene_interface ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES}
# moveit_py_bindings_tools)
# set_target_properties(moveit_planning_scene_interface_python PROPERTIES
# OUTPUT_NAME _moveit_planning_scene_interface PREFIX "")
# set_target_properties(moveit_planning_scene_interface_python PROPERTIES
# LIBRARY_OUTPUT_DIRECTORY "bin")
if(WIN32)
  # set_target_properties(moveit_planning_scene_interface_python PROPERTIES
  # SUFFIX .pyd)
endif()

# install(TARGETS moveit_planning_scene_interface_python EXPORT
# moveit_planning_scene_interface_python DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning_interface)
