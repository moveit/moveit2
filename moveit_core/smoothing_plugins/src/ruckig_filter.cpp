#include <moveit/smoothing_plugins/ruckig_filter.h>

namespace smoothing_plugins
{
bool RuckigFilterPlugin::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                                    size_t num_joints)
{
  return true;
};

bool RuckigFilterPlugin::doSmoothing(std::vector<double>& position_vector)
{
  return true;
};

bool RuckigFilterPlugin::reset(const std::vector<double>& joint_positions)
{
  return true;
};

}  // namespace smoothing_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smoothing_plugins::RuckigFilterPlugin, smoothing_plugins::SmoothingBaseClass)
