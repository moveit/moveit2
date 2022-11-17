/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

/* Author: David V. Lu!! */

#include <moveit/rdf_loader/rdf_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

TEST(RDFIntegration, default_arguments)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("default_arguments");
  rdf_loader::RDFLoader loader(node);
  ASSERT_NE(nullptr, loader.getURDF());
  EXPECT_EQ("kermit", loader.getURDF()->name_);
  ASSERT_NE(nullptr, loader.getSRDF());
  EXPECT_EQ("kermit", loader.getSRDF()->getName());
}

TEST(RDFIntegration, non_existent)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("non_existent");
  rdf_loader::RDFLoader loader(node, "does_not_exist");
  ASSERT_EQ(nullptr, loader.getURDF());
  ASSERT_EQ(nullptr, loader.getSRDF());
}

TEST(RDFIntegration, topic_based)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("topic_based");
  rdf_loader::RDFLoader loader(node, "topic_description");
  ASSERT_NE(nullptr, loader.getURDF());
  EXPECT_EQ("gonzo", loader.getURDF()->name_);
  ASSERT_NE(nullptr, loader.getSRDF());
  EXPECT_EQ("gonzo", loader.getSRDF()->getName());
}

TEST(RDFIntegration, executor)
{
  // RDFLoader should successfully load URDF and SRDF strings from a ROS topic when the node that is
  // passed to it is spinning.
  // GIVEN a node that has been added to an executor that is spinning on another thread
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("executor");

  // Create a thread to spin an Executor.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinning_thread([&executor] { executor.spin(); });

  // WHEN the RDFLoader is created
  rdf_loader::RDFLoader loader(node, "topic_description");

  // THEN the RDFLoader should return non-null values for the URDF and SRDF model.
  ASSERT_NE(nullptr, loader.getURDF());
  EXPECT_EQ("gonzo", loader.getURDF()->name_);
  ASSERT_NE(nullptr, loader.getSRDF());
  EXPECT_EQ("gonzo", loader.getSRDF()->getName());
  executor.cancel();
  spinning_thread.join();
}

TEST(RDFIntegration, xacro_test)
{
  std::string output;
  std::vector<std::string> xacro_args;
  ASSERT_TRUE(rdf_loader::RDFLoader::loadPkgFileToString(output, "moveit_ros_planning",
                                                         "rdf_loader/test/data/robin.srdf.xacro", xacro_args));
  EXPECT_GT(output.size(), 0u);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
