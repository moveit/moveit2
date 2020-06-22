/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/profiler/profiler.h>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("inverse_kinematics_test");
static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("inverse_kinematics_test");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  if (argc <= 1)
    RCLCPP_ERROR(LOGGER, "An argument specifying the group name is needed");
  else
  {
    robot_model_loader::RobotModelLoader rml(node, ROBOT_DESCRIPTION);
    std::string group = argv[1];
    RCLCPP_INFO(LOGGER, "Evaluating IK for %s", group.c_str());

    const moveit::core::JointModelGroup* jmg = rml.getModel()->getJointModelGroup(group);
    if (jmg)
    {
      const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
      if (solver)
      {
        const std::string& tip = solver->getTipFrame();
        moveit::core::RobotState state(rml.getModel());
        state.setToDefaultValues();

        RCLCPP_INFO(LOGGER, "Tip Frame:  %s", solver->getTipFrame().c_str());
        RCLCPP_INFO(LOGGER, "Base Frame: %s", solver->getBaseFrame().c_str());
        RCLCPP_INFO(LOGGER, "IK Timeout: %f", solver->getDefaultTimeout());
        RCLCPP_INFO(LOGGER, "Search res: %f", solver->getSearchDiscretization());

        unsigned int test_count = 1000;
        if (argc > 2)
          try
          {
            test_count = boost::lexical_cast<unsigned int>(argv[2]);
          }
          catch (...)
          {
          }

        RCLCPP_INFO(LOGGER, "Running %u tests", test_count);

        moveit::tools::Profiler::Start();
        for (unsigned int i = 0; i < test_count; ++i)
        {
          state.setToRandomPositions(jmg);
          // getGlobalLinkTransform() returns a valid isometry by contract
          Eigen::Isometry3d pose = state.getGlobalLinkTransform(tip);
          state.setToRandomPositions(jmg);
          moveit::tools::Profiler::Begin("IK");
          state.setFromIK(jmg, pose);
          moveit::tools::Profiler::End("IK");
          // getGlobalLinkTransform() returns a valid isometry by contract
          const Eigen::Isometry3d& pose_upd = state.getGlobalLinkTransform(tip);
          Eigen::Isometry3d diff = pose_upd * pose.inverse();  // valid isometry
          double rot_err = (diff.linear() - Eigen::Matrix3d::Identity()).norm();
          double trans_err = diff.translation().norm();
          moveit::tools::Profiler::Average("Rotation error", rot_err);
          moveit::tools::Profiler::Average("Translation error", trans_err);
          if (rot_err < 1e-3 && trans_err < 1e-3)
          {
            moveit::tools::Profiler::Event("Valid IK");
            moveit::tools::Profiler::Average("Success Rate", 100);
          }
          else
          {
            moveit::tools::Profiler::Event("Invalid IK");
            moveit::tools::Profiler::Average("Success Rate", 0);
          }
        }
        moveit::tools::Profiler::Stop();
        moveit::tools::Profiler::Status();
      }
      else
        RCLCPP_ERROR(LOGGER, "No kinematics solver specified for group %s", group.c_str());
    }
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
