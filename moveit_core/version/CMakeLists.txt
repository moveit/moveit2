# Generate and install version.h

if(NOT "$ENV{USER}" STREQUAL "buildfarm") # Don't define version postfix on
                                          # buildfarm
  set(MOVEIT_VERSION_EXTRA
      "devel"
      CACHE STRING "version string postfix")
endif()

message(
  STATUS
    " *** Building MoveIt ${moveit_core_VERSION} ${MOVEIT_VERSION_EXTRA} ***")

# https://stackoverflow.com/questions/13920072/how-to-always-run-command-when-building-regardless-of-any-dependency
add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/include/moveit/version.h always_rebuild
  COMMENT "Generate version header"
  COMMAND
    ${CMAKE_COMMAND} -DVERSION_FILE_PATH="${CMAKE_BINARY_DIR}/include"
    -DMOVEIT_VERSION="${moveit_core_VERSION}"
    -DMOVEIT_VERSION_EXTRA="${MOVEIT_VERSION_EXTRA}" -P
    ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(
  version_h
  DEPENDS always_rebuild
  COMMENT "Generating version.h")

add_executable(moveit_version version.cpp)
add_dependencies(moveit_version version_h)

target_include_directories(moveit_version
                           PRIVATE ${CMAKE_BINARY_DIR}/include/moveit_core)

install(FILES "${CMAKE_BINARY_DIR}/include/moveit_core/moveit/version.h"
        DESTINATION include/moveit_core/moveit)
install(TARGETS moveit_version RUNTIME DESTINATION bin)
