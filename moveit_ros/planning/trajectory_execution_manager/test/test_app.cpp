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

#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_trajectory_execution_manager");

  robot_model_loader::RobotModelLoaderPtr rml(new robot_model_loader::RobotModelLoader(node));

  printf("robot_model_loader::RobotModelLoaderPtr\n");

  planning_scene_monitor::PlanningSceneMonitor psm(rml, node);
  printf("planning_scene_monitor::PlanningSceneMonitor\n");
  trajectory_execution_manager::TrajectoryExecutionManager tem(rml->getModel(), psm.getStateMonitor(), true, node);
  printf("trajectory_execution_manager::TrajectoryExecutionManager\n");

  std::cout << "1:\n";
  if (!tem.ensureActiveControllersForJoints(std::vector<std::string>(1, "motor1")))
    printf("Fail!\n");

  // std::cout << "2:\n";
  // if (!tem.ensureActiveController("arms"))
  //   printf("Fail!\n");
  //
  // std::cout << "3:\n";
  // if (!tem.ensureActiveControllersForJoints(std::vector<std::string>(1, "rj2")))
  //   printf("Fail!\n");
  //
  // std::cout << "4:\n";
  // if (!tem.ensureActiveControllersForJoints(std::vector<std::string>(1, "lj1")))
  //   printf("Fail!\n");
  //
  // std::cout << "5:\n";
  // if (!tem.ensureActiveController("left_arm_head"))
  //   printf("Fail!\n");
  // std::cout << "6:\n";
  // if (!tem.ensureActiveController("arms"))
  //   printf("Fail!\n");
  //
  // // execute with empty set of trajectories
  // tem.execute();
  // if (!tem.waitForExecution())
  //   printf("Fail!\n");
  //
  // moveit_msgs::msg::RobotTrajectory traj1;
  // traj1.joint_trajectory.joint_names.push_back("rj1");
  // traj1.joint_trajectory.points.resize(1);
  // traj1.joint_trajectory.points[0].positions.push_back(0.0);
  // if (!tem.push(traj1))
  //   printf("Fail!\n");
  //
  // moveit_msgs::msg::RobotTrajectory traj2 = traj1;
  // traj2.joint_trajectory.joint_names.push_back("lj2");
  // traj2.joint_trajectory.points[0].positions.push_back(1.0);
  // traj2.multi_dof_joint_trajectory.joint_names.push_back("basej");
  // traj2.multi_dof_joint_trajectory.points.resize(1);
  // traj2.multi_dof_joint_trajectory.points[0].transforms.resize(1);
  //
  // if (!tem.push(traj2))
  //   printf("Fail!\n");
  //
  // traj1.multi_dof_joint_trajectory = traj2.multi_dof_joint_trajectory;
  // if (!tem.push(traj1))
  //   printf("Fail!\n");
  //
  // if (!tem.executeAndWait())
  //   printf("Fail!\n");

  return 0;
}
