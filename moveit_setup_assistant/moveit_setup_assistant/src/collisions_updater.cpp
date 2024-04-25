/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fraunhofer IPA
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
 *   * Neither the name of Fraunhofer IPA nor the names of its
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

/* Author: Mathias Lüdtke */

#include <moveit_setup_srdf_plugins/default_collisions.hpp>
#include <moveit_setup_framework/data/package_settings_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit/rdf_loader/rdf_loader.h>
#include <boost/program_options.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace po = boost::program_options;

int main(int argc, char* argv[])
{
  std::filesystem::path config_pkg_path;
  std::filesystem::path urdf_path;
  std::filesystem::path srdf_path;
  std::filesystem::path output_path;

  bool include_default = false, include_always = false, keep_old = false, verbose = false;

  double min_collision_fraction = 1.0;

  uint32_t never_trials = 0;

  // clang-format off
  po::options_description desc("Allowed options");
  desc.add_options()("help", "show help")("config-pkg", po::value(&config_pkg_path), "path to MoveIt config package")(
      "urdf", po::value(&urdf_path), "path to URDF ( or xacro)")("srdf", po::value(&srdf_path),
                                                                 "path to SRDF ( or xacro)")(
      "output", po::value(&output_path),
      "output path for SRDF")("xacro-args", po::value<std::vector<std::string> >()->composing(),
                              "additional arguments for xacro")("default", po::bool_switch(&include_default),
                                                                "disable default colliding pairs")(
      "always", po::bool_switch(&include_always), "disable always colliding pairs")("keep", po::bool_switch(&keep_old),
                                                                                    "keep disabled link from SRDF")(
      "verbose", po::bool_switch(&verbose), "verbose output")("trials", po::value(&never_trials),
                                                              "number of trials for searching never colliding pairs")(
      "min-collision-fraction", po::value(&min_collision_fraction),
      "fraction of small sample size to determine links that are always colliding");
  // clang-format on

  po::positional_options_description pos_desc;
  pos_desc.add("xacro-args", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << '\n';
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("collision_updater");
  moveit_setup::DataWarehousePtr config_data = std::make_shared<moveit_setup::DataWarehouse>(node);

  moveit_setup::srdf_setup::DefaultCollisions setup_step;
  setup_step.initialize(node, config_data);

  if (!config_pkg_path.empty())
  {
    auto package_settings = config_data->get<moveit_setup::PackageSettingsConfig>("package_settings");
    try
    {
      package_settings->loadExisting(config_pkg_path);
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not load config at '" << config_pkg_path << "'. " << e.what());
      return 1;
    }
  }
  else if (urdf_path.empty() || srdf_path.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Please provide config package or URDF and SRDF path");
    return 1;
  }
  else if (rdf_loader::RDFLoader::isXacroFile(srdf_path) && output_path.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Please provide a different output file for SRDF xacro input file");
    return 1;
  }

  auto srdf_config = config_data->get<moveit_setup::SRDFConfig>("srdf");

  // overwrite config paths if applicable
  if (!urdf_path.empty())
  {
    auto config = config_data->get<moveit_setup::URDFConfig>("urdf");

    std::vector<std::string> xacro_args;
    if (vm.count("xacro-args"))
      xacro_args = vm["xacro-args"].as<std::vector<std::string> >();

    config->loadFromPath(urdf_path, xacro_args);
  }
  if (!srdf_path.empty())
  {
    srdf_config->loadSRDFFile(srdf_path);
  }

  if (!keep_old)
  {
    srdf_config->clearCollisionData();
  }

  setup_step.startGenerationThread(never_trials, min_collision_fraction, verbose);
  int thread_progress;
  int last_progress = 0;
  while ((thread_progress = setup_step.getThreadProgress()) < 100)
  {
    if (thread_progress - last_progress > 10)
    {
      RCLCPP_INFO(node->get_logger(), "%d%% complete...", thread_progress);
      last_progress = thread_progress;
    }
  }
  setup_step.joinGenerationThread();
  RCLCPP_INFO(node->get_logger(), "100%% complete...");

  size_t skip_mask = 0;
  if (!include_default)
    skip_mask |= (1 << moveit_setup::srdf_setup::DEFAULT);
  if (!include_always)
    skip_mask |= (1 << moveit_setup::srdf_setup::ALWAYS);

  setup_step.linkPairsToSRDFSorted(skip_mask);

  srdf_config->write(output_path.empty() ? srdf_config->getPath() : output_path);

  return 0;
}
