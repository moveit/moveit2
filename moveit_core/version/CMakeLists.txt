add_executable(moveit_version version.cpp)
target_include_directories(moveit_version PRIVATE ${VERSION_FILE_PATH})
install(TARGETS moveit_version RUNTIME DESTINATION bin)
