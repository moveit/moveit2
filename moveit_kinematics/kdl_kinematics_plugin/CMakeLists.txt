generate_parameter_library(
  kdl_kinematics_parameters # cmake target name for the parameter library
  src/kdl_kinematics_parameters.yaml # path to input yaml file
)

add_library(moveit_kdl_kinematics_plugin SHARED
            src/kdl_kinematics_plugin.cpp src/chainiksolver_vel_mimic_svd.cpp)

target_link_libraries(
  moveit_kdl_kinematics_plugin
  kdl_kinematics_parameters
  rclcpp::rclcpp
  random_numbers::random_numbers
  pluginlib::pluginlib
  moveit_core::moveit_core
  ${moveit_msgs_TARGETS}
  ${orocos_kdl_LIBRARIES}
  kdl_parser::kdl_parser
  tf2_kdl::tf2_kdl
  Eigen3::Eigen)

# prevent pluginlib from using boost
target_compile_definitions(moveit_kdl_kinematics_plugin
                           PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(moveit_kdl_kinematics_plugin
                           PRIVATE "MOVEIT_KDL_KINEMATICS_PLUGIN_BUILDING_DLL")

install(DIRECTORY include/ DESTINATION include/moveit_kinematics)
