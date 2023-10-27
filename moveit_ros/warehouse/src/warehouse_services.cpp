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

/* Author: Dan Greenwald */

#include <moveit/warehouse/state_storage.h>
#include <moveit_msgs/srv/save_robot_state_to_warehouse.hpp>
#include <moveit_msgs/srv/list_robot_states_in_warehouse.hpp>
#include <moveit_msgs/srv/get_robot_state_from_warehouse.hpp>
#include <moveit_msgs/srv/check_if_robot_state_exists_in_warehouse.hpp>
#include <moveit_msgs/srv/delete_robot_state_from_warehouse.hpp>
#include <moveit_msgs/srv/rename_robot_state_in_warehouse.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/utilities.hpp>
<<<<<<< HEAD
=======
#include <moveit/utils/logger.hpp>

using moveit::getLogger;
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))

static const std::string ROBOT_DESCRIPTION = "robot_description";

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros.warehouse.warehouse_services");

bool storeState(const std::shared_ptr<moveit_msgs::srv::SaveRobotStateToWarehouse::Request>& request,
                const std::shared_ptr<moveit_msgs::srv::SaveRobotStateToWarehouse::Response>& response,
                moveit_warehouse::RobotStateStorage& rs)
{
  if (request->name.empty())
  {
<<<<<<< HEAD
    RCLCPP_ERROR(LOGGER, "You must specify a name to store a state");
=======
    RCLCPP_ERROR(getLogger(), "You must specify a name to store a state");
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
    return (response->success = false);
  }
  rs.addRobotState(request->state, request->name, request->robot);
  return (response->success = true);
}

bool listStates(const std::shared_ptr<moveit_msgs::srv::ListRobotStatesInWarehouse::Request>& request,
                const std::shared_ptr<moveit_msgs::srv::ListRobotStatesInWarehouse::Response>& response,
                moveit_warehouse::RobotStateStorage& rs)
{
  if (request->regex.empty())
  {
    rs.getKnownRobotStates(response->states, request->robot);
  }
  else
  {
    rs.getKnownRobotStates(request->regex, response->states, request->robot);
  }
  return true;
}

bool hasState(const std::shared_ptr<moveit_msgs::srv::CheckIfRobotStateExistsInWarehouse::Request>& request,
              const std::shared_ptr<moveit_msgs::srv::CheckIfRobotStateExistsInWarehouse::Response>& response,
              moveit_warehouse::RobotStateStorage& rs)
{
  response->exists = rs.hasRobotState(request->name, request->robot);
  return true;
}

bool getState(const std::shared_ptr<moveit_msgs::srv::GetRobotStateFromWarehouse::Request>& request,
              const std::shared_ptr<moveit_msgs::srv::GetRobotStateFromWarehouse::Response>& response,
              moveit_warehouse::RobotStateStorage& rs)
{
  if (!rs.hasRobotState(request->name, request->robot))
  {
<<<<<<< HEAD
    RCLCPP_ERROR_STREAM(LOGGER, "No state called '" << request->name << "' for robot '" << request->robot << "'.");
=======
    RCLCPP_ERROR_STREAM(getLogger(), "No state called '" << request->name << "' for robot '" << request->robot << "'.");
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
    moveit_msgs::msg::RobotState dummy;
    response->state = dummy;
    return false;
  }
  moveit_warehouse::RobotStateWithMetadata state_buffer;
  rs.getRobotState(state_buffer, request->name, request->robot);
  response->state = static_cast<const moveit_msgs::msg::RobotState&>(*state_buffer);
  return true;
}

bool renameState(const std::shared_ptr<moveit_msgs::srv::RenameRobotStateInWarehouse::Request>& request,
                 const std::shared_ptr<moveit_msgs::srv::RenameRobotStateInWarehouse::Response>& /*response*/,
                 moveit_warehouse::RobotStateStorage& rs)
{
  if (!rs.hasRobotState(request->old_name, request->robot))
  {
<<<<<<< HEAD
    RCLCPP_ERROR_STREAM(LOGGER, "No state called '" << request->old_name << "' for robot '" << request->robot << "'.");
=======
    RCLCPP_ERROR_STREAM(getLogger(),
                        "No state called '" << request->old_name << "' for robot '" << request->robot << "'.");
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
    return false;
  }
  rs.renameRobotState(request->old_name, request->new_name, request->robot);
  return true;
}

bool deleteState(const std::shared_ptr<moveit_msgs::srv::DeleteRobotStateFromWarehouse::Request>& request,
                 const std::shared_ptr<moveit_msgs::srv::DeleteRobotStateFromWarehouse::Response>& /*response*/,
                 moveit_warehouse::RobotStateStorage& rs)
{
  if (!rs.hasRobotState(request->name, request->robot))
  {
<<<<<<< HEAD
    RCLCPP_ERROR_STREAM(LOGGER, "No state called '" << request->name << "' for robot '" << request->robot << "'.");
=======
    RCLCPP_ERROR_STREAM(getLogger(), "No state called '" << request->name << "' for robot '" << request->robot << "'.");
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
    return false;
  }
  rs.removeRobotState(request->name, request->robot);
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_warehouse_services", node_options);
<<<<<<< HEAD
=======
  moveit::getLoggerMut() = node->get_logger();
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))

  std::string host;

  int port;
  double connection_timeout;
  int connection_retries;

  node->get_parameter_or(std::string("warehouse_host"), host, std::string("localhost"));
  node->get_parameter_or(std::string("warehouse_port"), port, 33829);
  node->get_parameter_or(std::string("warehouse_db_connection_timeout"), connection_timeout, 5.0);
  node->get_parameter_or(std::string("warehouse_db_connection_retries"), connection_retries, 5);

  warehouse_ros::DatabaseConnection::Ptr conn;

  try
  {
    conn = moveit_warehouse::loadDatabase(node);
    conn->setParams(host, port, connection_timeout);

<<<<<<< HEAD
    RCLCPP_INFO(LOGGER, "Connecting to warehouse on %s:%d", host.c_str(), port);
=======
    RCLCPP_INFO(getLogger(), "Connecting to warehouse on %s:%d", host.c_str(), port);
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
    int tries = 0;
    while (!conn->connect())
    {
      ++tries;
<<<<<<< HEAD
      RCLCPP_WARN(LOGGER, "Failed to connect to DB on %s:%d (try %d/%d).", host.c_str(), port, tries,
                  connection_retries);
      if (tries == connection_retries)
      {
        RCLCPP_FATAL(LOGGER, "Failed to connect too many times, giving up");
=======
      RCLCPP_WARN(getLogger(), "Failed to connect to DB on %s:%d (try %d/%d).", host.c_str(), port, tries,
                  connection_retries);
      if (tries == connection_retries)
      {
        RCLCPP_FATAL(getLogger(), "Failed to connect too many times, giving up");
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
        return 1;
      }
    }
  }
  catch (std::exception& ex)
  {
<<<<<<< HEAD
    RCLCPP_ERROR(LOGGER, "%s", ex.what());
=======
    RCLCPP_ERROR(getLogger(), "%s", ex.what());
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
    return 1;
  }

  moveit_warehouse::RobotStateStorage rs(conn);

  std::vector<std::string> names;
  rs.getKnownRobotStates(names);
  if (names.empty())
  {
<<<<<<< HEAD
    RCLCPP_INFO(LOGGER, "There are no previously stored robot states");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Previously stored robot states:");
    for (const std::string& name : names)
      RCLCPP_INFO(LOGGER, " * %s", name.c_str());
=======
    RCLCPP_INFO(getLogger(), "There are no previously stored robot states");
  }
  else
  {
    RCLCPP_INFO(getLogger(), "Previously stored robot states:");
    for (const std::string& name : names)
      RCLCPP_INFO(getLogger(), " * %s", name.c_str());
>>>>>>> 63e0c3a39 (Add new clang-tidy style rules (#2177))
  }

  auto save_cb = [&](const std::shared_ptr<moveit_msgs::srv::SaveRobotStateToWarehouse::Request>& request,
                     const std::shared_ptr<moveit_msgs::srv::SaveRobotStateToWarehouse::Response>& response) -> bool {
    return storeState(request, response, rs);
  };

  auto list_cb = [&](const std::shared_ptr<moveit_msgs::srv::ListRobotStatesInWarehouse::Request>& request,
                     const std::shared_ptr<moveit_msgs::srv::ListRobotStatesInWarehouse::Response>& response) -> bool {
    return listStates(request, response, rs);
  };

  auto get_cb = [&](const std::shared_ptr<moveit_msgs::srv::GetRobotStateFromWarehouse::Request>& request,
                    const std::shared_ptr<moveit_msgs::srv::GetRobotStateFromWarehouse::Response>& response) -> bool {
    return getState(request, response, rs);
  };

  auto has_cb =
      [&](const std::shared_ptr<moveit_msgs::srv::CheckIfRobotStateExistsInWarehouse::Request>& request,
          const std::shared_ptr<moveit_msgs::srv::CheckIfRobotStateExistsInWarehouse::Response>& response) -> bool {
    return hasState(request, response, rs);
  };

  auto rename_cb =
      [&](const std::shared_ptr<moveit_msgs::srv::RenameRobotStateInWarehouse::Request>& request,
          const std::shared_ptr<moveit_msgs::srv::RenameRobotStateInWarehouse::Response>& response) -> bool {
    return renameState(request, response, rs);
  };

  auto delete_cb =
      [&](const std::shared_ptr<moveit_msgs::srv::DeleteRobotStateFromWarehouse::Request>& request,
          const std::shared_ptr<moveit_msgs::srv::DeleteRobotStateFromWarehouse::Response>& response) -> bool {
    return deleteState(request, response, rs);
  };

  auto save_state_server =
      node->create_service<moveit_msgs::srv::SaveRobotStateToWarehouse>("save_robot_state", save_cb);
  auto list_states_server =
      node->create_service<moveit_msgs::srv::ListRobotStatesInWarehouse>("list_robot_states", list_cb);
  auto get_state_server = node->create_service<moveit_msgs::srv::GetRobotStateFromWarehouse>("get_robot_state", get_cb);
  auto has_state_server =
      node->create_service<moveit_msgs::srv::CheckIfRobotStateExistsInWarehouse>("has_robot_state", has_cb);
  auto rename_state_server =
      node->create_service<moveit_msgs::srv::RenameRobotStateInWarehouse>("rename_robot_state", rename_cb);
  auto delete_state_server =
      node->create_service<moveit_msgs::srv::DeleteRobotStateFromWarehouse>("delete_robot_state", delete_cb);

  rclcpp::spin(node);
  return 0;
}
