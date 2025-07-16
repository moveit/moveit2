cmake_minimum_required(VERSION 3.22)
project(moveit_ros_perception LANGUAGES CXX)

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

option(WITH_OPENGL "Build the parts that depend on OpenGL" ON)

if(WITH_OPENGL)
  # Prefer newer vendor-specific OpenGL library
  if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
  endif()
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)

  set(GL_LIBS ${GL_LIBS} ${OPENGL_LIBRARIES})
  if(APPLE)
    # Manually set GLEW and GLUT paths for Homebrew on macOS
    set(GLEW_INCLUDE_DIR "/opt/homebrew/opt/glew/include")
    set(GLEW_LIBRARY "/opt/homebrew/opt/glew/lib/libGLEW.dylib")
    set(GLUT_INCLUDE_DIR "/opt/homebrew/opt/freeglut/include")
    set(GLUT_LIBRARY "/opt/homebrew/lib/libglut.dylib")
    # find_package(FreeGLUT REQUIRED)
    set(GLEW_LIBRARIES ${GLEW_LIBRARY})
    set(GLEW_INCLUDE_DIRS ${GLEW_INCLUDE_DIR})
    set(GLUT_LIBRARIES ${GLUT_LIBRARY})
    set(GLUT_INCLUDE_DIRS ${GLUT_INCLUDE_DIR})

    set(SYSTEM_GL_INCLUDE_DIR ${GLEW_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
    set(SYSTEM_GL_LIBRARIES ${GLEW_LIBRARIES} ${GLUT_LIBRARIES})
  else()
    find_package(GLUT REQUIRED)
    if(WIN32)
      set(SYSTEM_GL_LIBRARIES GLEW::glew GLUT::GLUT)
    else()
      set(SYSTEM_GL_LIBRARIES ${GLEW_LIBRARIES} GLUT::GLUT)
    endif()
  endif()
  set(PERCEPTION_GL_INCLUDE_DIRS "mesh_filter/include"
                                 "depth_image_octomap_updater/include")
  set(SYSTEM_GL_INCLUDE_DIRS ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
endif()

if(APPLE)
  find_package(X11 REQUIRED)
endif()

# MacOS Threads lib fix
if(APPLE)
  set(CMAKE_THREAD_LIBS_INIT "-lpthread")
  set(CMAKE_HAVE_THREADS_LIBRARY 1)
  set(CMAKE_USE_WIN32_THREADS_INIT 0)
  set(CMAKE_USE_PTHREADS_INIT 1)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(image_transport REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV)

# Finds Boost Components
include(ConfigExtras.cmake)

set(THIS_PACKAGE_INCLUDE_DIRS
    depth_image_octomap_updater/include
    lazy_free_space_updater/include
    mesh_filter/include
    point_containment_filter/include
    pointcloud_octomap_updater/include
    semantic_world/include
    ${PERCEPTION_GL_INCLUDE_DIRS})

set(THIS_PACKAGE_LIBRARIES
    moveit_depth_image_octomap_updater
    moveit_depth_image_octomap_updater_core
    moveit_lazy_free_space_updater
    moveit_mesh_filter
    moveit_point_containment_filter
    moveit_pointcloud_octomap_updater
    moveit_pointcloud_octomap_updater_core
    moveit_semantic_world)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    image_transport
    moveit_core
    moveit_msgs
    moveit_ros_occupancy_map_monitor
    object_recognition_msgs
    rclcpp
    sensor_msgs
    tf2_geometry_msgs
    Eigen3)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(
  SYSTEM ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${SYSTEM_GL_INCLUDE_DIR} ${X11_INCLUDE_DIR})

add_subdirectory(lazy_free_space_updater)
add_subdirectory(point_containment_filter)
add_subdirectory(pointcloud_octomap_updater)
if(WITH_OPENGL)
  add_subdirectory(mesh_filter)
  add_subdirectory(depth_image_octomap_updater)
endif()

add_subdirectory(semantic_world)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT moveit_ros_perceptionTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_perception)

ament_export_targets(moveit_ros_perceptionTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  "pointcloud_octomap_updater_plugin_description.xml")
pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  "depth_image_octomap_updater_plugin_description.xml")

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
