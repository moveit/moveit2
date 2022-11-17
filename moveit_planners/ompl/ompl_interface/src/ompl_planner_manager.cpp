/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Dave Coleman */

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ompl/util/Console.h>

namespace ompl_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ompl_planning.ompl_planner_manager");
static const rclcpp::Logger OMPL_LOGGER = rclcpp::get_logger("ompl");

class OMPLPlannerManager : public planning_interface::PlannerManager
{
public:
  OMPLPlannerManager() : planning_interface::PlannerManager()
  {
    class OutputHandler : public ompl::msg::OutputHandler
    {
    public:
      void log(const std::string& text, ompl::msg::LogLevel level, const char* filename, int line) override
      {
        switch (level)
        {
          case ompl::msg::LOG_DEV2:
          case ompl::msg::LOG_DEV1:
          case ompl::msg::LOG_DEBUG:
          case ompl::msg::LOG_INFO:
            // LOG_INFO too verbose for MoveIt usage, so we reduce the logger level to DEBUG
            RCLCPP_DEBUG(OMPL_LOGGER, "%s:%i - %s", filename, line, text.c_str());
            break;
          case ompl::msg::LOG_WARN:
            RCLCPP_WARN(OMPL_LOGGER, "%s:%i - %s", filename, line, text.c_str());
            break;
          case ompl::msg::LOG_ERROR:
            RCLCPP_ERROR(OMPL_LOGGER, "%s:%i - %s", filename, line, text.c_str());
            break;
          case ompl::msg::LOG_NONE:
          default:
            /* ignore */
            break;
        }
      }
    };

    output_handler_ = std::make_shared<OutputHandler>();
    ompl::msg::useOutputHandler(output_handler_.get());
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    ompl_interface_ = std::make_unique<OMPLInterface>(model, node, parameter_namespace);
    setPlannerConfigurations(ompl_interface_->getPlannerConfigurations());
    return true;
  }

  bool canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const override
  {
    return req.trajectory_constraints.constraints.empty();
  }

  std::string getDescription() const override
  {
    return "OMPL";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    const planning_interface::PlannerConfigurationMap& pconfig = ompl_interface_->getPlannerConfigurations();
    algs.clear();
    algs.reserve(pconfig.size());
    for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
      algs.push_back(config.first);
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) override
  {
    // this call can add a few more configs than we pass in (adds defaults)
    ompl_interface_->setPlannerConfigurations(pconfig);
    // so we read the configs instead of just setting pconfig
    PlannerManager::setPlannerConfigurations(ompl_interface_->getPlannerConfigurations());
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<OMPLInterface> ompl_interface_;
  std::shared_ptr<ompl::msg::OutputHandler> output_handler_;
};

}  // namespace ompl_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ompl_interface::OMPLPlannerManager, planning_interface::PlannerManager)
