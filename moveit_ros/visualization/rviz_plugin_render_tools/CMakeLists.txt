set(MOVEIT_LIB_NAME moveit_rviz_plugin_render_tools)

set(HEADERS
  include/moveit/rviz_plugin_render_tools/octomap_render.h
  include/moveit/rviz_plugin_render_tools/planning_link_updater.h
  include/moveit/rviz_plugin_render_tools/planning_scene_render.h
  include/moveit/rviz_plugin_render_tools/render_shapes.h
  include/moveit/rviz_plugin_render_tools/robot_state_visualization.h
  include/moveit/rviz_plugin_render_tools/trajectory_visualization.h
  include/moveit/rviz_plugin_render_tools/trajectory_panel.h
  include/ogre_helpers/mesh_shape.hpp
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${MOVEIT_LIB_NAME} SHARED
  src/render_shapes.cpp
  src/robot_state_visualization.cpp
  src/planning_scene_render.cpp
  src/planning_link_updater.cpp
  src/octomap_render.cpp
  src/trajectory_visualization.cpp
  src/trajectory_panel.cpp
  src/mesh_shape.cpp
  ${HEADERS}
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
    Qt5::Widgets
)

ament_target_dependencies(${MOVEIT_LIB_NAME}
    rclcpp
    moveit_core
    Boost
    octomap_msgs
    rviz_ogre_vendor
    rviz_common
    rviz_default_plugins
)
target_include_directories(${MOVEIT_LIB_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include)
