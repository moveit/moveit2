cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

option(WITH_OPENGL "Build the parts that depend on OpenGL" ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

if(WITH_OPENGL)
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(GLUT REQUIRED)
  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
  set(perception_GL_INCLUDE_DIRS "mesh_filter/include" "depth_image_octomap_updater/include")
  set(SYSTEM_GL_INCLUDE_DIRS ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
endif(WITH_OPENGL)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(ament_cmake REQUIRED)
find_package(ros1_bridge REQUIRED)
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


find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(OpenCV)

set(THIS_PACKAGE_INCLUDE_DIRS
  lazy_free_space_updater/include
  point_containment_filter/include
  occupancy_map_monitor/include
  pointcloud_octomap_updater/include
  semantic_world/include
  ${perception_GL_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_LIBRARIES
    # moveit_lazy_free_space_updater
    # moveit_point_containment_filter
    moveit_occupancy_map_monitor
    # moveit_pointcloud_octomap_updater_core
    # moveit_semantic_world
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    moveit_core
    image_transport
    moveit_msgs
    sensor_msgs
    object_recognition_msgs
    tf2_geometry_msgs
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${SYSTEM_GL_INCLUDE_DIRS}
    ${X11_INCLUDE_DIR}
    ${rclcpp_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIR}
    ${octomap_msgs_INCLUDE_DIR}
)

include_directories(lazy_free_space_updater/include
                    point_containment_filter/include
                    occupancy_map_monitor/include
                    pointcloud_octomap_updater/include
                    semantic_world/include
                    ${perception_GL_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${rclcpp_INCLUDE_DIRS}
                    ${moveit_msgs_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    include
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${SYSTEM_GL_INCLUDE_DIR}
                    ${OPENGL_INCLUDE_DIR}
                    ${X11_INCLUDE_DIR}
                    ${moveit_core_INCLUDE_DIRS}
                    )


# add_subdirectory(lazy_free_space_updater)
# add_subdirectory(point_containment_filter)
add_subdirectory(occupancy_map_monitor)
# add_subdirectory(pointcloud_octomap_updater)
# if (WITH_OPENGL)
  # add_subdirectory(mesh_filter)
  # add_subdirectory(depth_image_octomap_updater)
# endif(WITH_OPENGL)

# add_subdirectory(semantic_world)

install(
  FILES
    pointcloud_octomap_updater_plugin_description.xml
    depth_image_octomap_updater_plugin_description.xml
  DESTINATION
    share)

ament_export_include_directories(include)
ament_export_libraries(${THIS_PACKAGE_INCLUDE_LIBRARIES})

ament_package()
