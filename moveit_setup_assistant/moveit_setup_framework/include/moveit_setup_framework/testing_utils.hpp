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

#pragma once

#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_framework/setup_step.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <filesystem>

namespace moveit_setup
{
/**
 * @brief Test environment with DataWarehouse setup and help for generating files in a temp dir
 */
class MoveItSetupTest : public ::testing::Test
{
protected:
  /**
   * @brief Initialize the node, DataWarehouse and output dir.
   *
   * You may want to override this method to register data types after calling MoveItSetupTest::SetUp()
   */
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_node");
    config_data_ = std::make_shared<moveit_setup::DataWarehouse>(node_);
    output_dir_ = std::filesystem::temp_directory_path() / "moveit_setup_test";
  }

  /**
   * @brief Helper function for generating all the files for a particular config to a temporary directory
   *
   * Uses the same template/config_name structure as DataWarehouse::get
   */
  template <typename T>
  void generateFiles(const std::string& config_name)
  {
    std::vector<moveit_setup::GeneratedFilePtr> files;
    config_data_->get<T>(config_name)->collectFiles(output_dir_, placeholder_timestamp_, files);
    for (moveit_setup::GeneratedFilePtr& file : files)
    {
      file->write();
    }
  }

  void initializeStep(SetupStep& setup_step)
  {
    setup_step.initialize(node_, config_data_);
  }

  /**
   * @brief Clean up by removing all files when complete
   *
   * Note that you only want to set delete_when_finished_ to false when testing locally
   */
  void TearDown() override
  {
    if (delete_when_finished_)
      std::filesystem::remove_all(output_dir_);
  }

  rclcpp::Node::SharedPtr node_;
  moveit_setup::DataWarehousePtr config_data_;
  std::filesystem::path output_dir_;
  moveit_setup::GeneratedTime placeholder_timestamp_;
  bool delete_when_finished_{ true };  // Set to false to keep the files around
};

std::set<std::string> getKeys(const YAML::Node& node)
{
  std::set<std::string> keys;
  for (const auto& kv : node)
  {
    keys.insert(kv.first.as<std::string>());
  }
  return keys;
}

void expectYamlEquivalence(const YAML::Node& generated, const YAML::Node& reference,
                           const std::filesystem::path& generated_path, const std::string& yaml_namespace = "")
{
  std::string msg_prefix =
      std::string("In ") + generated_path.c_str() + ", node with namespace '" + yaml_namespace + "' ";

  if (generated.Type() != reference.Type())
  {
    ADD_FAILURE() << msg_prefix + "does not have matching types!";
    return;
  }

  if (generated.IsSequence())
  {
    if (generated.size() != reference.size())
    {
      ADD_FAILURE() << msg_prefix + "does not have matching sizes!";
      return;
    }
    for (std::size_t i = 0; i < generated.size(); ++i)
    {
      std::string sub_namespace = yaml_namespace + "[" + std::to_string(i) + "]";
      expectYamlEquivalence(generated[i], reference[i], generated_path, sub_namespace);
    }
  }
  else if (generated.IsMap())
  {
    std::set<std::string> gkeys = getKeys(generated), rkeys = getKeys(reference), missing_keys, common_keys, extra_keys;

    std::set_difference(rkeys.begin(), rkeys.end(), gkeys.begin(), gkeys.end(),
                        std::inserter(missing_keys, missing_keys.end()));
    std::set_difference(gkeys.begin(), gkeys.end(), rkeys.begin(), rkeys.end(),
                        std::inserter(extra_keys, extra_keys.end()));
    std::set_intersection(gkeys.begin(), gkeys.end(), rkeys.begin(), rkeys.end(),
                          std::inserter(common_keys, common_keys.end()));

    for (const std::string& key : missing_keys)
    {
      ADD_FAILURE() << msg_prefix << "is missing the key '" << key << "'";
    }
    for (const std::string& key : extra_keys)
    {
      ADD_FAILURE() << msg_prefix << "has an extra key '" << key << "'";
    }
    for (const std::string& key : common_keys)
    {
      std::string sub_namespace = yaml_namespace + "/" + key;
      expectYamlEquivalence(generated[key], reference[key], generated_path, sub_namespace);
    }
  }
  else
  {
    EXPECT_EQ(generated.as<std::string>(), reference.as<std::string>())
        << msg_prefix << "does not match scalar values!";
  }
}

void expectYamlEquivalence(const std::filesystem::path& generated_path, const std::filesystem::path& reference_path)
{
  EXPECT_EQ(std::filesystem::is_regular_file(generated_path), std::filesystem::is_regular_file(reference_path));
  if (!std::filesystem::is_regular_file(reference_path))
  {
    return;
  }
  const YAML::Node& generated = YAML::LoadFile(generated_path);
  const YAML::Node& reference = YAML::LoadFile(reference_path);
  expectYamlEquivalence(generated, reference, generated_path);
}

}  // namespace moveit_setup
