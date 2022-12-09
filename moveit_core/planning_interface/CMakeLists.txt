target_sources(moveit_core PRIVATE
  src/planning_interface.cpp
  src/planning_response.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
