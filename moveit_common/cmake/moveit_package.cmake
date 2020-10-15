macro(moveit_package)
  if(NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 14)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # Enable warnings
    add_compile_options(-Wall -Wextra
      -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual
      -Wno-unused-parameter -Wno-unused-function)
  else()
    # Defaults for Microsoft C++ compiler
    add_compile_options(/W3 /wd4251 /wd4068 /wd4275)

    # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

    # Enable Math Constants
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    add_compile_definitions(
      _USE_MATH_DEFINES
    )
  endif()

  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # This too often has false-positives
    add_compile_options(-Wno-maybe-uninitialized)
  endif()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
    set(CMAKE_BUILD_TYPE Release)
  endif()
endmacro()
