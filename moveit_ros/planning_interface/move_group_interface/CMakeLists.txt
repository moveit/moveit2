add_library(moveit_move_group_interface SHARED src/move_group_interface.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_move_group_interface)
target_include_directories(moveit_move_group_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_move_group_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_move_group_interface moveit_common_planning_interface_objects moveit_planning_scene_interface ${Boost_THREAD_LIBRARY})
ament_target_dependencies(moveit_move_group_interface
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_occupancy_map_monitor
  moveit_ros_planning
  moveit_ros_warehouse
)

# TODO (ddengster) : port wrap_python_move_group
#add_library(moveit_move_group_interface_python src/wrap_python_move_group.cpp)
#target_link_libraries(moveit_move_group_interface_python moveit_move_group_interface ${eigenpy_LIBRARIES} ${PYTHON_LIBRARIES} ${LIBS} ${Boost_LIBRARIES} moveit_py_bindings_tools)
#add_dependencies(moveit_move_group_interface_python)
#set_target_properties(moveit_move_group_interface_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
#set_target_properties(moveit_move_group_interface_python PROPERTIES OUTPUT_NAME _moveit_move_group_interface PREFIX "")
#set_target_properties(moveit_move_group_interface_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
if(WIN32)
#  set_target_properties(moveit_move_group_interface_python PROPERTIES SUFFIX .pyd)
endif()

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning_interface)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_move_group_interface_export.h DESTINATION include/moveit_ros_planning_interface)

#add_executable(demo src/demo.cpp)
#target_link_libraries(demo moveit_move_group_interface ${LIBS} ${Boost_LIBRARIES})
