add_library(moveit_macros INTERFACE)
target_include_directories(moveit_macros INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
