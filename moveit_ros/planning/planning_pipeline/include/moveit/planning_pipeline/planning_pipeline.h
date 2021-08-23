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

#pragma once

#include <atomic>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

#include <moveit_planning_pipeline_export.h>

/** \brief Planning pipeline */
namespace planning_pipeline
{
constexpr char PLANNING_PLUGIN_PARAM_NAME[] = "planning_plugin";
constexpr char ADAPTER_PLUGINS_PARAM_NAME[] = "request_adapters";

/** \brief This class facilitates loading planning plugins and
    planning request adapted plugins.  and allows calling
    planning_interface::PlanningContext::solve() from a loaded
    planning plugin and the
    planning_request_adapter::PlanningRequestAdapter plugins, in the
    specified order. */
class MOVEIT_PLANNING_PIPELINE_EXPORT PlanningPipeline
{
public:
  /**
   * @brief      This class contains the rcl interfaces for easier testing
   */
  class MiddlewareHandle
  {
  public:
    /**
     * @brief      Destroys the object.
     */
    virtual ~MiddlewareHandle() = default;

    /**
     * @brief     Check if parameter exists
     */
    virtual bool has_parameter(const std::string& name) const = 0;

    /**
     * @brief     Get parameter of specific plugin
     * @param[in] pluginName name of plugin that has parameter
     * @param[in] parameters container for parameters
     */
    virtual void get_parameters(const std::string& pluginName, std::string& parameters) const = 0;
    /**
     * @brief     Create a display path publisher
     * @param[in] topic the topic
     */
    virtual void createDisplayPathPublisher(const std::string& topic) = 0;

    /**
     * @brief     Create a received request publisher
     * @param[in] topic the topic
     */
    virtual void createReceivedRequestPublisher(const std::string& topic) = 0;

    /**
     * @brief     Create a contacts publisher
     * @param[in] topic the topic
     */
    virtual void createContactsPublisher(const std::string& topic) = 0;

    /**
     * @brief     Get topic name of Contacts Publisher
     */

    virtual std::string getContactsTopicName() const = 0;

    /**
     * @brief     Reset a display path publisher
     */
    virtual void resetDisplayPathPublisher() = 0;

    /**
     * @brief     Reset a received request publisher
     */
    virtual void resetReceivedRequestPublisher() = 0;

    /**
     * @brief     Reset a contacts publisher
     */
    virtual void resetContactsPublisher() = 0;

    /**
     * @brief     Create a planner plugin
     * @param[in] name plugin name
     */
    virtual void createPlannerPlugin(const std::string& name, const moveit::core::RobotModelConstPtr& robot_model) = 0;]

    /**
     * @brief Retrieves a pointer to a planner manager
     */
    virtual const planning_interface::PlannerManagerPtr& getPlannerManager() const = 0;

    /**
     * @brief     Create a adapter plugin
     * @param[in] name plugin name
     */
    virtual void createAdapterPlugins(const std::vector<std::string>& name) = 0;

    /**
     * @brief
     */
    virtual bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                      std::vector<std::size_t>& adapter_added_state_index) = 0;

    /**
     * @brief     Terminate planner instance
     */
    virtual void terminatePlannerPlugin() = 0;

    /**
     * @brief     Publish received motion plan request
     * @param[in] req Received Motion Plan to be published
     */
    virtual void publishReceivedRequest(const planning_interface::MotionPlanRequest& req) = 0;

    /**
     * @brief     Publish contact markers
     * @param[in] markerArray Contact Marker Array to be published
     */
    virtual void publishContactsMarkerArray(const visualization_msgs::msg::MarkerArray& markerArray) = 0;

    /**
     * @brief     Publish Display Trejectory Path
     * @param[in] displayTrajectory Display Trajectory to be published
     */
    virtual void publishDisplayPathTrejectory(const moveit_msgs::msg::DisplayTrajectory& displayTrajectory) = 0;
  };

  /** \brief When motion plans are computed and they are supposed to be automatically displayed, they are sent to this
   * topic (moveit_msgs::msg::DisplayTrajectory) */
  static const std::string DISPLAY_PATH_TOPIC;

  /** \brief When motion planning requests are received and they are supposed to be automatically published, they are
   * sent to this topic (moveit_msgs::msg::MotionPlanRequest) */
  static const std::string MOTION_PLAN_REQUEST_TOPIC;

  /** \brief When contacts are found in the solution path reported by a planner, they can be published as markers on
   * this topic (visualization_msgs::MarkerArray) */
  static const std::string MOTION_CONTACTS_TOPIC;

  /** @brief Constructor.
   *  \param model The robot model for which this pipeline is initialized.
   *  @param rcl_interface   The rcl middleware interface
   *  \param parameter_namespace the parameter namespace where the planner configurations are stored
   *  \param planning_plugin_param_name The name of the ROS parameter under which the name of the planning plugin is
   *  specified
   *  \param adapter_plugins_param_name The name of the ROS parameter under which the names of the request adapter
   *  plugins are specified (plugin names separated by space; order matters)
   */
  PlanningPipeline(const moveit::core::RobotModelConstPtr& model, std::unique_ptr<MiddlewareHandle> middleware_handle,
                   const std::string& parameter_namespace,
                   const std::string& planning_plugin_param_name = PLANNING_PLUGIN_PARAM_NAME,
                   const std::string& adapter_plugins_param_name = ADAPTER_PLUGINS_PARAM_NAME);

  /** \brief Given a robot model (\e model), a node handle (\e pipeline_nh), initialize the planning pipeline.
      \param model The robot model for which this pipeline is initialized.
      \param node The ROS node that should be used for reading parameters needed for configuration
      \param parameter_namespace parameter namespace where the planner configurations are stored
      \param planning_plugin_param_name The name of the ROS parameter under which the name of the planning plugin is
     specified
      \param adapter_plugins_param_name The name of the ROS parameter under which the names of the request adapter
     plugins are specified (plugin names separated by space; order matters)
  */

  PlanningPipeline(const moveit::core::RobotModelConstPtr& model, const std::shared_ptr<rclcpp::Node>& node,
                   const std::string& parameter_namespace,
                   const std::string& planning_plugin_param_name = PLANNING_PLUGIN_PARAM_NAME,
                   const std::string& adapter_plugins_param_name = ADAPTER_PLUGINS_PARAM_NAME);

  /** @brief Constructor.
   *  \param model The robot model for which this pipeline is initialized.
   *  @param rcl_interface   The rcl middleware interface
      \param paramenter_namespace the parameter namespace where the planner configurations are stored
      \param planning_plugin_name The name of the planning plugin to load
      \param adapter_plugins_names The names of the planning request adapter plugins to load
   */
  PlanningPipeline(const moveit::core::RobotModelConstPtr& model, std::unique_ptr<MiddlewareHandle> middleware_handle,
                   const std::string& parameter_namespace, const std::string& planning_plugin_name,
                   const std::vector<std::string>& adapter_plugin_names);

  /** \brief Given a robot model (\e model), a node handle (\e pipeline_nh), initialize the planning pipeline.
      \param model The robot model for which this pipeline is initialized.
      \param node The ROS node that should be used for reading parameters needed for configuration
      \param paramenter_namespace the parameter namespace where the planner configurations are stored
      \param planning_plugin_name The name of the planning plugin to load
      \param adapter_plugins_names The names of the planning request adapter plugins to load
  */
  PlanningPipeline(const moveit::core::RobotModelConstPtr& model, const std::shared_ptr<rclcpp::Node>& node,
                   const std::string& parameter_namespace, const std::string& planning_plugin_name,
                   const std::vector<std::string>& adapter_plugin_names);

  /** \brief Pass a flag telling the pipeline whether or not to publish the computed motion plans on DISPLAY_PATH_TOPIC.
   * Default is true. */
  void displayComputedMotionPlans(bool flag);

  /** \brief Pass a flag telling the pipeline whether or not to publish the received motion planning requests on
   * MOTION_PLAN_REQUEST_TOPIC. Default is false. */
  void publishReceivedRequests(bool flag);

  /** \brief Pass a flag telling the pipeline whether or not to re-check the solution paths reported by the planner.
   * This is true by default.  */
  void checkSolutionPaths(bool flag);

  /** \brief Get the flag set by displayComputedMotionPlans() */
  bool getDisplayComputedMotionPlans() const
  {
    return is_displaying_computed_motion_plans_;
  }

  /** \brief Get the flag set by publishReceivedRequests() */
  bool getPublishReceivedRequests() const
  {
    return is_publishing_received_requests_;
  }

  /** \brief Get the flag set by checkSolutionPaths() */
  bool getCheckSolutionPaths() const
  {
    return is_checking_solution_paths_;
  }

  /** \brief Call the motion planner plugin and the sequence of planning request adapters (if any).
      \param planning_scene The planning scene where motion planning is to be done
      \param req The request for motion planning
      \param res The motion planning response */
  bool generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res) const;

  /** \brief Call the motion planner plugin and the sequence of planning request adapters (if any).
      \param planning_scene The planning scene where motion planning is to be done
      \param req The request for motion planning
      \param res The motion planning response
      \param adapter_added_state_index Sometimes planning request adapters may add states on the solution path (e.g.,
     add the current state of the robot as prefix, when the robot started to plan only from near that state, as the
     current state itself appears to touch obstacles). This is helpful because the added states should not be considered
     invalid in all situations. */
  bool generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& adapter_added_state_index) const;

  /** \brief Request termination, if a generatePlan() function is currently computing plans */
  void terminate() const;

  /** \brief Get the name of the planning plugin used */
  const std::string& getPlannerPluginName() const
  {
    return planner_plugin_name_;
  }

  /** \brief Get the names of the planning request adapter plugins used */
  const std::vector<std::string>& getAdapterPluginNames() const
  {
    return adapter_plugin_names_
  }

  /** \brief Get the planner manager for the loaded planning plugin */
  const planning_interface::PlannerManagerPtr& getPlannerManager()
  {
    return middleware_handle_->getPlannerManager();
  }

  /** \brief Get the robot model that this pipeline is using */
  const moveit::core::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  /** \brief Get current status of the planning pipeline */
  [[nodiscard]] bool isActive() const
  {
    return is_active_;
  }

private:
  void configure();

  // Flag that indicates whether or not the planning pipeline is currently solving a planning problem
  mutable std::atomic<bool> is_active_;

  std::shared_ptr<rclcpp::Node> node_;
  std::string parameter_namespace_;
  /// Flag indicating whether motion plans should be published as a moveit_msgs::msg::DisplayTrajectory
  bool is_displaying_computed_motion_plans_;

  /// Flag indicating whether received requests should be published just before beginning processing (useful for
  /// debugging)
  bool is_publishing_received_requests_;
  std::string planner_plugin_name_;
  std::vector<std::string> adapter_plugin_names_;

  moveit::core::RobotModelConstPtr robot_model_;

  /// Flag indicating whether the reported plans should be checked once again, by the planning pipeline itself
  bool is_checking_solution_paths_;
};

MOVEIT_CLASS_FORWARD(PlanningPipeline);  // Defines PlanningPipelinePtr, ConstPtr, WeakPtr... etc
}  // namespace planning_pipeline
