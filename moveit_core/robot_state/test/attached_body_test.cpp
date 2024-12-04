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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Tom Noble */

#include <geometric_shapes/shapes.h>
#include <moveit/robot_state/attached_body.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <urdf_parser/urdf_parser.h>
#include <gtest/gtest.h>

class SingleLinkRobot : public ::testing::Test
{
public:
  virtual void SetUp() override
  {
    std::cout << "SingleLinkRobot" << std::endl;
    static const std::string URDF_XML = R"(
      <?xml version="1.0" ?>
      <robot name="single_link_robot">
      <link name="link">
        <inertial>
          <mass value="2.81"/>
          <origin rpy="0 0 0" xyz="0.0 0.0 .0"/>
          <inertia ixx="0.1" ixy="-0.2" ixz="0.5" iyy="-.09" iyz="1" izz="0.101"/>
        </inertial>
        <collision name="my_collision">
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <geometry>
            <box size="1 2 1" />
          </geometry>
        </collision>
        <visual>
          <origin rpy="0 0 0" xyz="0.0 0 0"/>
          <geometry>
            <box size="1 2 1" />
          </geometry>
        </visual>
      </link>
      <joint name="joint" type="revolute">
          <axis xyz="0 0 1"/>
          <parent link="link"/>
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <limit effort="100.0" lower="-3.14" upper="3.14" velocity="0.2"/>
      </joint>
      </robot>
    )";

    static const std::string SRDF_XML = R"xml(
      <?xml version="1.0" ?>
      <robot name="single_link_robot">
        <group name="joint_group">
          <joint name="joint"/>
        </group>
      </robot>
      )xml";

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(URDF_XML);
    auto srdf_model = std::make_shared<srdf::Model>();
    srdf_model->initString(*urdf_model, SRDF_XML);
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    std::cout << robot_model_->getName() << std::endl;
  }

  virtual void TearDown() override
  {
  }

protected:
  moveit::core::RobotModelConstPtr robot_model_;
};

class SingleAttachedBody : public SingleLinkRobot
{
protected:
  virtual void SetUp() override
  {
    std::cout << "SingleAttachedBody" << std::endl;
    const moveit::core::LinkModel* link = robot_model_->getLinkModel("link");
    std::string name = "root_body";
    Eigen::Isometry3d pose;
    pose = Eigen::Translation3d(1, 2, 3);
    auto box = std::make_shared<shapes::Box>(0.1, 0.2, 0.3);
    std::vector<shapes::ShapeConstPtr> shapes = { box };
    EigenSTL::vector_Isometry3d shape_poses;
    Eigen::Isometry3d shape_pose;
    shape_pose = Eigen::Translation3d(4, 5, 6);
    shape_poses.push_back(shape_pose);
    std::set<std::string> touch_links = { "link" };
    trajectory_msgs::msg::JointTrajectory detach_posture;
    detach_posture.joint_names.push_back("joint");
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.push_back(0.1);
    detach_posture.points.push_back(p);
    Eigen::Isometry3d subframe_pose;
    subframe_pose = Eigen::Translation3d(7, 8, 9);
    moveit::core::FixedTransformsMap subframes{ { "subframe", subframe_pose } };
    root_body_ = std::make_shared<moveit::core::AttachedBody>(link, name, pose, shapes, shape_poses, touch_links,
                                                              detach_posture, subframes);
  }

  virtual void TearDown() override
  {
  }

protected:
  std::shared_ptr<moveit::core::AttachedBody> root_body_;
};

// Verifies that a single body attached to a link works as intended

TEST_F(SingleAttachedBody, RootBodyHasCorrecAttachedLink)
{
  ASSERT_EQ(root_body_->getName(), "root_body");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
