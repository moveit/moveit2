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

/* Author: Ioan Sucan */

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/profiler/profiler.h>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <memory>

static const std::string PLANNER_NODE_NAME = "ompl_planning";  // name of node
static const std::string PLANNER_SERVICE_NAME =
    "plan_kinematic_path";  // name of the advertised service (within the ~ namespace)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

bool shutdown_req = false;

class OMPLPlannerService
{
public:
  OMPLPlannerService(planning_scene_monitor::PlanningSceneMonitor& psm, bool debug = false,
                        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ompl_planner"))
    : node_(node), psm_(psm), ompl_interface_(psm.getPlanningScene()->getRobotModel(),node), debug_(debug)
  {
    std::function<bool( std::shared_ptr<rmw_request_id_t>,
                     std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request>,
                     std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response>)> cb_get_plan_service = std::bind(
       &OMPLPlannerService::computePlan, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

    plan_service_ = node_->create_service<moveit_msgs::srv::GetMotionPlan>(PLANNER_SERVICE_NAME,
                                                                                        cb_get_plan_service);
    if (debug_)
    {
      pub_plan_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("display_motion_plan", 100);
      pub_request_ = node_->create_publisher<moveit_msgs::msg::MotionPlanRequest>("motion_plan_request", 100);
    }
  }

  bool computePlan(std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request> req,
    std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> res)
  {
    RCLCPP_INFO(node_->get_logger(),"Received new planning request...");
    if (debug_)
      pub_request_->publish(req->motion_plan_request);
    planning_interface::MotionPlanResponse response;

    ompl_interface::ModelBasedPlanningContextPtr context =
        ompl_interface_.getPlanningContext(psm_.getPlanningScene(), req->motion_plan_request);
    if (!context)
    {
      RCLCPP_ERROR(node_->get_logger(), "No planning context found");
      return false;
    }
    context->clear();

    bool result = context->solve(response);

    if (debug_)
    {
      if (result)
        displaySolution(res->motion_plan_response);
      std::stringstream ss;
      RCLCPP_INFO(node_->get_logger(),"%s", ss.str().c_str());
    }
    return result;
  }

  void displaySolution(const moveit_msgs::msg::MotionPlanResponse& mplan_res)
  {
    moveit_msgs::msg::DisplayTrajectory d;
    d.model_id = psm_.getPlanningScene()->getRobotModel()->getName();
    d.trajectory_start = mplan_res.trajectory_start;
    d.trajectory.resize(1, mplan_res.trajectory);
    pub_plan_->publish(d);
  }

  void status()
  {
    ompl_interface_.printStatus();
    RCLCPP_INFO(node_->get_logger(),"Responding to planning and bechmark requests");
    if (debug_)
      RCLCPP_INFO(node_->get_logger(),"Publishing debug information");
  }

private:
  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitor& psm_;
  ompl_interface::OMPLInterface ompl_interface_;
  rclcpp::Service<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_service_;
  // rclcpp::Service<moveit_msgs::srv::GetMotionPlan>::SharedPtr display_states_service_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr pub_plan_;
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanRequest>::SharedPtr pub_request_;
  bool debug_;
};

void signalHandler(int signum)
{
  shutdown_req = true;
}

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(PLANNER_NODE_NAME);

  bool debug = false;
  for (int i = 1; i < argc; ++i)
    if (strncmp(argv[i], "--debug", 7) == 0)
      debug = true;
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf_buffer);
  if (psm.getPlanningScene())
  {
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();

    OMPLPlannerService pservice(psm, debug, node);

    while (!shutdown_req)
    {
      pservice.status();
      executor.spin_node_some(node);
    }
  }
  else
    RCLCPP_ERROR(node->get_logger(),"Planning scene not configured");

  return 0;
}
