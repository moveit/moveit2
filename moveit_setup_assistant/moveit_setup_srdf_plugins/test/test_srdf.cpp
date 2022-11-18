/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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
#include <moveit_setup_framework/testing_utils.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_srdf_plugins/planning_groups.hpp>
#include <tinyxml2.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_srdf");

using moveit_setup::getSharePath;
using moveit_setup::SRDFConfig;
using moveit_setup::srdf_setup::PlanningGroups;

class SRDFTest : public moveit_setup::MoveItSetupTest
{
protected:
  void SetUp() override
  {
    MoveItSetupTest::SetUp();
    srdf_config_ = config_data_->get<SRDFConfig>("srdf");
  }

  void initializeWithURDF(const std::filesystem::path& urdf)
  {
    config_data_->preloadWithURDFPath(urdf);
    srdf_config_->updateRobotModel();
  }

  void initializeWithFanuc()
  {
    initializeWithURDF(getSharePath("moveit_resources_fanuc_description") / "urdf" / "fanuc.urdf");
  }

  void generateXML(const std::string& robot_name = "fanuc")
  {
    generateFiles<SRDFConfig>("srdf");
    std::filesystem::path srdf_path = output_dir_ / "config" / (robot_name + ".srdf");
    ASSERT_TRUE(std::filesystem::is_regular_file(srdf_path));

    ASSERT_EQ(srdf_xml_.LoadFile(srdf_path.c_str()), tinyxml2::XML_SUCCESS);
  }

  std::shared_ptr<SRDFConfig> srdf_config_;
  tinyxml2::XMLDocument srdf_xml_;
};

unsigned int countElements(const tinyxml2::XMLNode& parent, const char* child_name)
{
  unsigned int count = 0;
  for (const tinyxml2::XMLElement* sub_el = parent.FirstChildElement(child_name); sub_el != nullptr;
       sub_el = sub_el->NextSiblingElement(child_name))
  {
    count++;
  }
  return count;
}

TEST_F(SRDFTest, Empty)
{
  initializeWithFanuc();

  // do nothing

  generateXML();

  auto root = srdf_xml_.FirstChildElement("robot");
  EXPECT_EQ(std::string(root->Attribute("name")), std::string("fanuc"));
  EXPECT_TRUE(root->NoChildren());
}

TEST_F(SRDFTest, SetJoints)
{
  initializeWithFanuc();

  PlanningGroups pg;
  initializeStep(pg);
  std::string group_name = "manipulator";
  pg.create(group_name);

  std::vector<std::string> joints = { "joint_1", "joint_2" };
  pg.setJoints(group_name, joints);

  generateXML();

  auto root = srdf_xml_.FirstChildElement("robot");
  auto group_el = root->FirstChildElement("group");
  ASSERT_NE(group_el, nullptr);
  EXPECT_EQ(std::string(group_el->Attribute("name")), group_name);
  EXPECT_EQ(countElements(*group_el, "joint"), 2u);

  joints.push_back("joint_3");
  joints.push_back("joint_4");
  pg.setJoints(group_name, joints);

  generateXML();

  root = srdf_xml_.FirstChildElement("robot");
  group_el = root->FirstChildElement("group");
  ASSERT_NE(group_el, nullptr);
  EXPECT_EQ(countElements(*group_el, "joint"), 4u);
}

TEST_F(SRDFTest, SetLinks)
{
  initializeWithFanuc();

  PlanningGroups pg;
  initializeStep(pg);
  std::string group_name = "manipulator";
  pg.create(group_name);

  std::vector<std::string> links = { "base_link", "link_1" };
  pg.setLinks(group_name, links);

  generateXML();

  auto root = srdf_xml_.FirstChildElement("robot");
  auto group_el = root->FirstChildElement("group");
  ASSERT_NE(group_el, nullptr);
  EXPECT_EQ(std::string(group_el->Attribute("name")), group_name);
  EXPECT_EQ(countElements(*group_el, "link"), 2u);

  links.push_back("link_2");
  links.push_back("link_3");
  pg.setLinks(group_name, links);

  generateXML();

  root = srdf_xml_.FirstChildElement("robot");
  group_el = root->FirstChildElement("group");
  ASSERT_NE(group_el, nullptr);
  EXPECT_EQ(countElements(*group_el, "link"), 4u);
}

TEST_F(SRDFTest, SetJointsThenLinks)
{
  initializeWithFanuc();

  PlanningGroups pg;
  initializeStep(pg);
  std::string group_name = "manipulator";
  pg.create(group_name);

  std::vector<std::string> joints = { "joint_1", "joint_2" };
  pg.setJoints(group_name, joints);

  generateXML();

  auto root = srdf_xml_.FirstChildElement("robot");
  auto group_el = root->FirstChildElement("group");
  ASSERT_NE(group_el, nullptr);
  EXPECT_EQ(std::string(group_el->Attribute("name")), group_name);
  EXPECT_EQ(countElements(*group_el, "joint"), 2u);
  EXPECT_EQ(countElements(*group_el, "link"), 0u);

  std::vector<std::string> links = { "base_link", "link_1" };
  pg.setLinks(group_name, links);

  generateXML();

  root = srdf_xml_.FirstChildElement("robot");
  group_el = root->FirstChildElement("group");
  ASSERT_NE(group_el, nullptr);
  EXPECT_EQ(std::string(group_el->Attribute("name")), group_name);
  EXPECT_EQ(countElements(*group_el, "joint"), 2u);
  EXPECT_EQ(countElements(*group_el, "link"), 2u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
