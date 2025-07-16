cmake_minimum_required(VERSION 3.22)
project(moveit_planners_ompl LANGUAGES CXX)

if(APPLE)
  # Manually tell CMake how to find OpenMP
  set(OpenMP_C_FLAGS "-Xclang -fopenmp")
  set(OpenMP_CXX_FLAGS "-Xclang -fopenmp")
  set(OpenMP_C_LIB_NAMES "omp")
  set(OpenMP_CXX_LIB_NAMES "omp")
  set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib")

  find_package(OpenMP REQUIRED)

  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP with Apple Clang enabled")
    add_compile_options(-Xclang -fopenmp)
    include_directories(SYSTEM "/opt/homebrew/opt/libomp/include")
    link_directories("/opt/homebrew/opt/libomp/lib")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lomp")
  else()
    message(FATAL_ERROR "OpenMP NOT found")
  endif()
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# MacOS Threads lib fix
if(APPLE)
  set(CMAKE_THREAD_LIBS_INIT "-lpthread")
  set(CMAKE_HAVE_THREADS_LIBRARY 1)
  set(CMAKE_USE_WIN32_THREADS_INIT 0)
  set(CMAKE_USE_PTHREADS_INIT 1)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
endif()

find_package(
  Boost
  REQUIRED
  system
  filesystem
  date_time
  thread
  serialization)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ompl REQUIRED)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wno-overloaded-virtual)
endif()

include_directories(ompl_interface/include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_subdirectory(ompl_interface)

install(
  TARGETS moveit_ompl_interface moveit_ompl_planner_plugin
  EXPORT moveit_planners_omplTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_planners_ompl)
ament_export_targets(moveit_planners_omplTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core ompl)

pluginlib_export_plugin_description_file(moveit_core
                                         ompl_interface_plugin_description.xml)

ament_package()
