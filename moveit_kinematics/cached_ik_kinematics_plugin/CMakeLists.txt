find_package(rsl REQUIRED)
find_package(trac_ik_kinematics_plugin QUIET)
find_package(ur_kinematics QUIET)

add_library(moveit_cached_ik_kinematics_base SHARED src/ik_cache.cpp)
set_target_properties(moveit_cached_ik_kinematics_base
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_cached_ik_kinematics_base PUBLIC rclcpp
                          moveit_core moveit_msgs)

if(trac_ik_kinematics_plugin_FOUND)
  include_directories(${trac_ik_kinematics_plugin_INCLUDE_DIRS})
endif()

generate_parameter_library(
  cached_ik_kinematics_parameters # cmake target name for the parameter library
  include/moveit/cached_ik_kinematics_plugin/cached_ik_kinematics_parameters.yaml # path to
                                                                                  # input yaml
                                                                                  # file
)

target_link_libraries(
  moveit_cached_ik_kinematics_base PUBLIC cached_ik_kinematics_parameters
                                          moveit_kdl_kinematics_plugin)

add_library(moveit_cached_ik_kinematics_plugin SHARED
            src/cached_ik_kinematics_plugin.cpp)
set_target_properties(moveit_cached_ik_kinematics_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_cached_ik_kinematics_plugin PUBLIC rclcpp
                          moveit_core moveit_msgs rsl)
target_link_libraries(
  moveit_cached_ik_kinematics_plugin
  PRIVATE cached_ik_kinematics_parameters moveit_kdl_kinematics_plugin
          moveit_srv_kinematics_plugin moveit_cached_ik_kinematics_base)
if(trac_ik_kinematics_plugin_FOUND)
  target_link_libraries(moveit_cached_ik_kinematics_plugin
                        PRIVATE ${trac_ik_kinematics_plugin_LIBRARIES})
  set_target_properties(
    moveit_cached_ik_kinematics_plugin
    PROPERTIES COMPILE_DEFINITIONS "CACHED_IK_KINEMATICS_TRAC_IK")
endif()

# This is just for testing purposes; the arms from Universal Robots have
# analytic solvers, so caching just adds extra overhead.
if(ur_kinematics_FOUND)
  include_directories(${ur_kinematics_INCLUDE_DIRS})
  foreach(ur_version 3 5 10)
    add_library(moveit_cached_ur${ur_version}_kinematics_plugin SHARED
                src/cached_ur_kinematics_plugin.cpp)
    set_target_properties(moveit_cached_ur${ur_version}_kinematics_plugin
                          PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
    find_library(ur${ur_version}_pluginlib ur${ur_version}_moveit_plugin
                 PATHS ${ur_kinematics_LIBRARY_DIRS})
    target_link_libraries(
      moveit_cached_ur${ur_version}_kinematics_plugin
      moveit_cached_ik_kinematics_base ${ur${ur_version}_pluginlib})
    install(
      TARGETS moveit_cached_ur${ur_version}_kinematics_plugin
      EXPORT moveit_cached_ur${ur_version}_kinematics_plugin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin)
  endforeach()
endif()

install(DIRECTORY include/ DESTINATION include/moveit_kinematics)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
