/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/*      Title     : TODO(adamp)
 *      Project   : moveit_servo
 *      Created   : 07/24/2020
 *      Author    : Adam Pettinger
 *      Desc      : Loads a Servo instance without launching or parameters
 */

#include <gtest/gtest.h>
#include <moveit_servo/servo.h>

#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem/path.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "test_parameter_struct.hpp"

void loadModelFile(std::string filename, std::string& file_content)
{
  boost::filesystem::path res_path(ament_index_cpp::get_package_share_directory("moveit_resources"));
  std::string xml_string;
  std::fstream xml_file((res_path / filename).string().c_str(), std::fstream::in);
  EXPECT_TRUE(xml_file.is_open());
  while (xml_file.good())
  {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  file_content = xml_string;
}

std::vector<std::string> joint_names_{"panda_finger_joint1", "panda_finger_joint2", "panda_joint1", "panda_joint2"
    "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

TEST(MOVEIT_SERVO, InitialTest)
{
  auto node = std::make_shared<rclcpp::Node>("test_servo");
  std::string robot_description_string, srdf_string;
  loadModelFile("panda_description/urdf/panda.urdf", robot_description_string);
  loadModelFile("panda_moveit_config/config/panda.srdf", srdf_string);

  node->declare_parameter<std::string>("robot_description", robot_description_string);
  node->declare_parameter<std::string>("robot_description_semantic", srdf_string);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description", tf_buffer, "planning_scene_monitor");

  RCLCPP_WARN_STREAM(node->get_logger(), "Test Point 1");

  EXPECT_TRUE(planning_scene_monitor->getPlanningScene());
  planning_scene_monitor->startStateMonitor("/joint_states");
  RCLCPP_WARN_STREAM(node->get_logger(), "Test Point 2");

  moveit_servo::ServoParametersPtr test_params = getTestParameters();
  RCLCPP_WARN_STREAM(node->get_logger(), "Test Point 3");

  moveit_servo::Servo servo(node, test_params, planning_scene_monitor);
  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto init_joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  init_joint_state_msg.get()->header.stamp = node->now();
  RCLCPP_WARN_STREAM(node->get_logger(), "Test Point 4");

  auto returned_params = servo.getParameters();
  RCLCPP_WARN_STREAM(node->get_logger(), "Test Point 5");

  EXPECT_TRUE(*returned_params == *test_params);
  RCLCPP_WARN_STREAM(node->get_logger(), "Test Point 6");

}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

// TEST(MOVEIT_SERVO, FilterReset)
// {
//   moveit_servo::LowPassFilter lpf(2.0);
//   EXPECT_DOUBLE_EQ(0.0, lpf.filter(0.0));
//   lpf.reset(5.0);
//   double value = lpf.filter(5.0);
//   EXPECT_DOUBLE_EQ(5.0, value);
//   EXPECT_NE(5.0, lpf.filter(100.0));
// }