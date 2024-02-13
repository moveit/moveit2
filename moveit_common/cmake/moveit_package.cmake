# Copyright 2021 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the PickNik Inc. nor the names of its contributors may
#   be used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Module for common settings in MoveIt packages.
macro(MOVEIT_PACKAGE)
  # Set ${PROJECT_NAME}_VERSION
  find_package(ament_cmake REQUIRED)
  ament_package_xml()

  # Enable backward_ros on every moveit package
  find_package(backward_ros QUIET)

  if(NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 17)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Enable warnings
    add_compile_options(
      -Wall
      -Wextra
      -Wwrite-strings
      -Wunreachable-code
      -Wpointer-arith
      -Wredundant-decls
      -Wcast-qual
      -Wold-style-cast
      -Wformat=2)
  else()
    # Defaults for Microsoft C++ compiler
    add_compile_options(/W3 /wd4251 /wd4068 /wd4275)

    # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

    # Enable Math Constants
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    add_compile_definitions(_USE_MATH_DEFINES)
  endif()

  option(MOVEIT_CI_WARNINGS "Enable all warnings used by CI" ON)
  if(MOVEIT_CI_WARNINGS)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      add_compile_options(
        -Wall
        -Wextra
        -Wwrite-strings
        -Wunreachable-code
        -Wpointer-arith
        -Wredundant-decls
        -Wcast-qual)
      # This too often has false-positives
      add_compile_options(-Wno-maybe-uninitialized)
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      add_compile_options(
        -Wall
        -Wextra
        -Wwrite-strings
        -Wunreachable-code
        -Wpointer-arith
        -Wredundant-decls
        -Wcast-qual)
    endif()
  endif()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    message(
      "${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance"
    )
    set(CMAKE_BUILD_TYPE Release)
  endif()
endmacro()
