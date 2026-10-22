#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace moveit_rviz_plugin
{
namespace detail
{

inline std::string getDefaultPlanningPipeline(const rclcpp::Node::SharedPtr& node)
{
  static constexpr char DEFAULT_PLANNING_PIPELINE_PARAMETER[] = "default_planning_pipeline";

  if (!node->has_parameter(DEFAULT_PLANNING_PIPELINE_PARAMETER))
    node->declare_parameter<std::string>(DEFAULT_PLANNING_PIPELINE_PARAMETER, "");

  std::string default_planning_pipeline;
  node->get_parameter(DEFAULT_PLANNING_PIPELINE_PARAMETER, default_planning_pipeline);
  return default_planning_pipeline;
}

}  // namespace detail
}  // namespace moveit_rviz_plugin
