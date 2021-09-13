#include <moveit_servo/smoothing_plugin/low_pass_filter.h>

namespace moveit_servo
{

bool initialize(moveit::core::RobotModelConstPtr robot_model, const size_t num_joints)
{
  return true;
};

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_servo::LowPassFilter, moveit_servo::SmoothingBaseClass)
