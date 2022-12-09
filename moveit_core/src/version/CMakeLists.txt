configure_file(version.h.in ${PROJECT_BINARY_DIR}/include/moveit/version.h)

add_executable(moveit_version version.cpp)
target_include_directories(moveit_version PRIVATE ${PROJECT_BINARY_DIR}/include)

install(TARGETS moveit_version RUNTIME DESTINATION bin)
install(FILES ${PROJECT_BINARY_DIR}/include/moveit/version.h DESTINATION include/moveit_core/moveit)
