/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group/move_group_capability.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <boost/tokenizer.hpp>
#include <moveit/macros/console_colors.hpp>
#include <moveit/move_group/move_group_context.hpp>
#include <memory>
#include <set>
#include <moveit/utils/logger.hpp>

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

namespace move_group
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.ros.move_group.executable");
}
}  // namespace

// These capabilities are loaded unless listed in disable_capabilities
// clang-format off
static const char* const DEFAULT_CAPABILITIES[] = {
   "move_group/LoadGeometryFromFileService",
   "move_group/SaveGeometryToFileService",
   "move_group/GetUrdfService",
   "move_group/MoveGroupCartesianPathService",
   "move_group/MoveGroupKinematicsService",
   "move_group/MoveGroupExecuteTrajectoryAction",
   "move_group/MoveGroupMoveAction",
   "move_group/MoveGroupPlanService",
   "move_group/MoveGroupQueryPlannersService",
   "move_group/MoveGroupStateValidationService",
   "move_group/MoveGroupGetPlanningSceneService",
   "move_group/ApplyPlanningSceneService",
   "move_group/ClearOctomapService",
};
// clang-format on

class MoveGroupExe
{
public:
  MoveGroupExe(const moveit_cpp::MoveItCppPtr& moveit_cpp, const std::string& default_planning_pipeline, bool debug)
  {
    // if the user wants to be able to disable execution of paths, they can just set this ROS param to false
    bool allow_trajectory_execution;
    moveit_cpp->getNode()->get_parameter_or("allow_trajectory_execution", allow_trajectory_execution, true);

    context_ =
        std::make_shared<MoveGroupContext>(moveit_cpp, default_planning_pipeline, allow_trajectory_execution, debug);

    // start the capabilities
    configureCapabilities();
  }

  ~MoveGroupExe()
  {
    capabilities_.clear();
    context_.reset();
    capability_plugin_loader_.reset();
  }

  void status()
  {
    if (context_)
    {
      if (context_->status())
      {
        if (capabilities_.empty())
        {
          printf("\n" MOVEIT_CONSOLE_COLOR_BLUE
                 "move_group is running but no capabilities are loaded." MOVEIT_CONSOLE_COLOR_RESET "\n\n");
        }
        else
        {
          printf("\n" MOVEIT_CONSOLE_COLOR_GREEN "You can start planning now!" MOVEIT_CONSOLE_COLOR_RESET "\n\n");
        }
        fflush(stdout);
      }
    }
    else
      RCLCPP_ERROR(getLogger(), "No MoveGroup context created. Nothing will work.");
  }

  MoveGroupContextPtr getContext()
  {
    return context_;
  }

private:
  void configureCapabilities()
  {
    try
    {
      capability_plugin_loader_ = std::make_shared<pluginlib::ClassLoader<MoveGroupCapability>>(
          "moveit_ros_move_group", "move_group::MoveGroupCapability");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL_STREAM(getLogger(),
                          "Exception while creating plugin loader for move_group capabilities: " << ex.what());
      return;
    }

    std::set<std::string> capabilities;

    // add default capabilities
    for (const char* capability : DEFAULT_CAPABILITIES)
      capabilities.insert(capability);

    // add capabilities listed in ROS parameter
    std::string capability_plugins;
    if (context_->moveit_cpp_->getNode()->get_parameter("capabilities", capability_plugins))
    {
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
      capabilities.insert(tok.begin(), tok.end());
    }

    // add capabilities configured for planning pipelines
    for (const auto& pipeline_entry : context_->moveit_cpp_->getPlanningPipelines())
    {
      const auto& pipeline_name = pipeline_entry.first;
      std::string pipeline_capabilities;
      if (context_->moveit_cpp_->getNode()->get_parameter(pipeline_name + ".capabilities", pipeline_capabilities))
      {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char>> tok(pipeline_capabilities, sep);
        capabilities.insert(tok.begin(), tok.end());
      }
    }

    // drop capabilities that have been explicitly disabled
    if (context_->moveit_cpp_->getNode()->get_parameter("disable_capabilities", capability_plugins))
    {
      boost::char_separator<char> sep(" ");
      boost::tokenizer<boost::char_separator<char>> tok(capability_plugins, sep);
      for (boost::tokenizer<boost::char_separator<char>>::iterator cap_name = tok.begin(); cap_name != tok.end();
           ++cap_name)
        capabilities.erase(*cap_name);
    }

    for (const std::string& capability : capabilities)
    {
      try
      {
        printf(MOVEIT_CONSOLE_COLOR_CYAN "Loading '%s'..." MOVEIT_CONSOLE_COLOR_RESET "\n", capability.c_str());
        MoveGroupCapabilityPtr cap = capability_plugin_loader_->createUniqueInstance(capability);
        cap->setContext(context_);
        cap->initialize();
        capabilities_.push_back(cap);
      }
      catch (pluginlib::PluginlibException& ex)
      {
        RCLCPP_ERROR_STREAM(getLogger(),
                            "Exception while loading move_group capability '" << capability << "': " << ex.what());
      }
    }

    std::stringstream ss;
    ss << '\n';
    ss << '\n';
    ss << "********************************************************" << '\n';
    ss << "* MoveGroup using: " << '\n';
    for (const MoveGroupCapabilityPtr& cap : capabilities_)
      ss << "*     - " << cap->getName() << '\n';
    ss << "********************************************************" << '\n';
    RCLCPP_INFO(getLogger(), "%s", ss.str().c_str());
  }

  MoveGroupContextPtr context_;
  std::shared_ptr<pluginlib::ClassLoader<MoveGroupCapability>> capability_plugin_loader_;
  std::vector<MoveGroupCapabilityPtr> capabilities_;
};
}  // namespace move_group

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opt;
  opt.allow_undeclared_parameters(true);
  opt.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("move_group", opt);
  moveit::setNodeLoggerName(nh->get_name());
  moveit_cpp::MoveItCpp::Options moveit_cpp_options(nh);

  // Prepare PlanningPipelineOptions
  moveit_cpp_options.planning_pipeline_options.parent_namespace = nh->get_effective_namespace() + ".planning_pipelines";
  std::vector<std::string> planning_pipeline_configs;
  if (nh->get_parameter("planning_pipelines", planning_pipeline_configs))
  {
    if (planning_pipeline_configs.empty())
    {
      RCLCPP_ERROR(nh->get_logger(), "Failed to read parameter 'move_group.planning_pipelines'");
    }
    else
    {
      for (const auto& config : planning_pipeline_configs)
      {
        moveit_cpp_options.planning_pipeline_options.pipeline_names.push_back(config);
      }
    }
  }

  // Retrieve default planning pipeline
  auto& pipeline_names = moveit_cpp_options.planning_pipeline_options.pipeline_names;
  std::string default_planning_pipeline;
  if (nh->get_parameter("default_planning_pipeline", default_planning_pipeline))
  {
    // Ignore default_planning_pipeline if there is no matching entry in pipeline_names
    if (std::find(pipeline_names.begin(), pipeline_names.end(), default_planning_pipeline) == pipeline_names.end())
    {
      RCLCPP_WARN(nh->get_logger(),
                  "MoveGroup launched with ~default_planning_pipeline '%s' not configured in ~planning_pipelines",
                  default_planning_pipeline.c_str());
      default_planning_pipeline = "";  // reset invalid pipeline id
    }
  }
  else if (pipeline_names.size() > 1)  // only warn if there are multiple pipelines to choose from
  {
    // Handle deprecated move_group.launch
    RCLCPP_WARN(nh->get_logger(),
                "MoveGroup launched without ~default_planning_pipeline specifying the namespace for the default "
                "planning pipeline configuration");
  }

  // If there is no valid default pipeline, either pick the first available one, or fall back to old behavior
  if (default_planning_pipeline.empty())
  {
    if (!pipeline_names.empty())
    {
      RCLCPP_WARN(nh->get_logger(), "Using default pipeline '%s'", pipeline_names[0].c_str());
      default_planning_pipeline = pipeline_names[0];
    }
    else
    {
      RCLCPP_WARN(nh->get_logger(), "Falling back to using the the move_group node namespace (deprecated behavior).");
      default_planning_pipeline = "move_group";
      moveit_cpp_options.planning_pipeline_options.pipeline_names = { default_planning_pipeline };
      moveit_cpp_options.planning_pipeline_options.parent_namespace = nh->get_effective_namespace();
    }

    // Reset invalid pipeline parameter for MGI requests
    nh->set_parameter(rclcpp::Parameter("default_planning_pipeline", default_planning_pipeline));
  }

  // Initialize MoveItCpp
  const auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(nh, moveit_cpp_options);
  const auto planning_scene_monitor = moveit_cpp->getPlanningSceneMonitorNonConst();

  if (planning_scene_monitor->getPlanningScene())
  {
    bool debug = false;
    for (int i = 1; i < argc; ++i)
    {
      if (strncmp(argv[i], "--debug", 7) == 0)
      {
        debug = true;
        break;
      }
    }
    debug = true;
    if (debug)
    {
      RCLCPP_INFO(nh->get_logger(), "MoveGroup debug mode is ON");
    }
    else
    {
      RCLCPP_INFO(nh->get_logger(), "MoveGroup debug mode is OFF");
    }

    rclcpp::executors::MultiThreadedExecutor executor;

    move_group::MoveGroupExe mge(moveit_cpp, default_planning_pipeline, debug);

    bool monitor_dynamics;
    if (nh->get_parameter("monitor_dynamics", monitor_dynamics) && monitor_dynamics)
    {
      RCLCPP_INFO(nh->get_logger(), "MoveGroup monitors robot dynamics (higher load)");
      planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);
    }

    planning_scene_monitor->publishDebugInformation(debug);

    mge.status();
    executor.add_node(nh);
    executor.spin();

    rclcpp::shutdown();
  }
  else
    RCLCPP_ERROR(nh->get_logger(), "Planning scene not configured");

  return 0;
}
