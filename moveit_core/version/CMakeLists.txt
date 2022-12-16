add_executable(moveit_version version.cpp)
target_include_directories(moveit_version PRIVATE ${CMAKE_BINARY_DIR}/include)
install(TARGETS moveit_version RUNTIME DESTINATION bin)
