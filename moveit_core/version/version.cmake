# Retrieve (active) branch name
execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE MOVEIT_GIT_NAME
  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

if("${MOVEIT_GIT_NAME}" STREQUAL "HEAD")
  # Retrieve any associated name (tag or branch)
  execute_process(
    COMMAND git describe --contains --all HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE MOVEIT_GIT_NAME
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()

# Retrieve (short) commit hash
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE MOVEIT_GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)

string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" MOVEIT_VERSION_MAJOR
                     "${moveit_core_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_MINOR
                     "${moveit_core_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" MOVEIT_VERSION_PATCH
                     "${moveit_core_VERSION}")
set(MOVEIT_VERSION
    "${MOVEIT_VERSION_MAJOR}.${MOVEIT_VERSION_MINOR}.${MOVEIT_VERSION_PATCH}")

if(NOT "${MOVEIT_VERSION_EXTRA}" STREQUAL "")
  string(APPEND MOVEIT_VERSION "-${MOVEIT_VERSION_EXTRA}")
endif()

configure_file("version.hpp.in"
               "${VERSION_FILE_PATH}/moveit_core/moveit/version.hpp")
