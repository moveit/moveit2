add_library(moveit_python_tools
  src/pybind_rosmsg_typecasters.cpp
)
#add_dependencies(moveit_python_tools ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_python_tools ${catkin_LIBRARIES}) # TODO: Fix
set_target_properties(moveit_python_tools PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(
  TARGETS
    moveit_python_tools
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/moveit_core)
