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
#include <moveit_setup_app_plugins/perception_config.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_perception");

using moveit_setup::expectYamlEquivalence;
using moveit_setup::getSharePath;
using moveit_setup::app::PerceptionConfig;

class PerceptionTest : public moveit_setup::MoveItSetupTest
{
protected:
  void SetUp() override
  {
    MoveItSetupTest::SetUp();
    config_data_->registerType("sensors", "moveit_setup::app::PerceptionConfig");
    templates_path_ = getSharePath("moveit_setup_app_plugins") / "templates";
    default_yaml_path_ = templates_path_ / "config/sensors_3d.yaml";
  }
  std::filesystem::path templates_path_;
  std::filesystem::path default_yaml_path_;
};

// This tests parsing of sensors_3d.yaml
TEST_F(PerceptionTest, ReadingSensorsConfig)
{
  auto sensors_config = config_data_->get<PerceptionConfig>("sensors");

  // Before parsing, no config available
  EXPECT_EQ(sensors_config->getSensorPluginConfig().size(), 0u);

  // Read the file containing the default config parameters
  auto configs = sensors_config->load3DSensorsYAML(default_yaml_path_);

  // Default config for the two available sensor plugins
  // Make sure both are parsed correctly
  ASSERT_EQ(configs.size(), 2u);

  EXPECT_EQ(configs[0]["sensor_plugin"], std::string("occupancy_map_monitor/PointCloudOctomapUpdater"));

  EXPECT_EQ(configs[1]["sensor_plugin"], std::string("occupancy_map_monitor/DepthImageOctomapUpdater"));
}

// This tests writing of sensors_3d.yaml
TEST_F(PerceptionTest, WritingSensorsConfig)
{
  auto sensors_config = config_data_->get<PerceptionConfig>("sensors");

  // Empty Config Should have No Sensors
  EXPECT_EQ(sensors_config->getSensorPluginConfig().size(), 0u);

  generateFiles<PerceptionConfig>("sensors");

  std::filesystem::path generated = output_dir_ / "config/sensors_3d.yaml";

  ASSERT_TRUE(std::filesystem::is_regular_file(generated));

  sensors_config->loadPrevious(std::filesystem::path("fake_path"), YAML::Node());

  // Should now have the defaults loaded
  EXPECT_EQ(sensors_config->getSensorPluginConfig().size(), 2u);

  generateFiles<PerceptionConfig>("sensors");

  expectYamlEquivalence(generated, default_yaml_path_);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
