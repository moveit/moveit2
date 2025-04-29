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

#include <moveit/macros/class_forward.hpp>
#include <moveit/utils/rclcpp_utils.hpp>
#include <moveit_ros_control_interface/ControllerHandle.hpp>
#include <moveit/controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_multiset_of.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>
#include <map>
#include <memory>
#include <queue>
#include <moveit/utils/logger.hpp>

static const rclcpp::Duration CONTROLLER_INFORMATION_VALIDITY_AGE = rclcpp::Duration::from_seconds(1.0);

namespace moveit_ros_control_interface
{
static constexpr double DEFAULT_SERVICE_CALL_TIMEOUT = 3.0;

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.plugins.ros_control_interface");
}
}  // namespace

/**
 * \brief Get joint name from resource name reported by ros2_control, since claimed_interfaces return by ros2_control
 * will have the interface name as suffix joint_name/INTERFACE_TYPE
 * @param[in] claimed_interface claimed interface as joint_name/INTERFACE_TYPE
 * @return joint_name part of the /p claimed_interface
 */
std::string parseJointNameFromResource(const std::string& claimed_interface)
{
  const auto index = claimed_interface.find('/');
  if (index == std::string::npos)
    return claimed_interface;
  return claimed_interface.substr(0, index);
}

/**
 * \brief  Modifies controller activation/deactivation lists to conform to ROS 2 control expectations.
 * \detail Activation/deactivation is expected to be disjoint. For example, if controller B is a dependency of A (A
 * chains to B) but controller B is also a dependency of C (B chains to B), then the switch from A->B to C->B would
 * cause B to be in both the activation and deactivate list. This causes ROS 2 control to throw an error and reject the
 * switch. This function adds the logic needed to avoid this from happening.
 * @param[in] activate_controllers  controllers to activate
 * @param[in] deactivate_controllers  controllers to deactivate
 */
void deconflictControllerActivationLists(std::vector<std::string>& activate_controllers,
                                         std::vector<std::string>& deactivate_controllers)
{
  // Convert vectors to sets for uniqueness
  std::unordered_set activate_set(activate_controllers.begin(), activate_controllers.end());
  std::unordered_set deactivate_set(deactivate_controllers.begin(), deactivate_controllers.end());

  // Find common elements
  std::unordered_set<std::string> common_controllers;
  for (const auto& str : activate_set)
  {
    if (deactivate_set.count(str))
    {
      common_controllers.insert(str);
    }
  }

  // Remove common elements from both sets
  for (const auto& controller_name : common_controllers)
  {
    activate_set.erase(controller_name);
    deactivate_set.erase(controller_name);
  }

  // Convert sets back to vectors
  activate_controllers.assign(activate_set.begin(), activate_set.end());
  deactivate_controllers.assign(deactivate_set.begin(), deactivate_set.end());
}

MOVEIT_CLASS_FORWARD(Ros2ControlManager);  // Defines Ros2ControlManagerPtr, ConstPtr, WeakPtr... etc

/**
 * \brief moveit_controller_manager::Ros2ControlManager sub class that interfaces one ros_control
 * controller_manager
 * instance.
 * All services and names are relative to ns_.
 */
class Ros2ControlManager : public moveit_controller_manager::MoveItControllerManager
{
  std::string ns_;
  std::chrono::duration<double> service_call_timeout_;
  pluginlib::ClassLoader<ControllerHandleAllocator> loader_;
  typedef std::map<std::string, controller_manager_msgs::msg::ControllerState> ControllersMap;

  /** @brief Controllers that can be activated and deactivated by this plugin. */
  ControllersMap managed_controllers_;
  /** @brief Controllers that are currently active. */
  ControllersMap active_controllers_;

  typedef std::map<std::string, ControllerHandleAllocatorPtr> AllocatorsMap;
  AllocatorsMap allocators_;

  typedef std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> HandleMap;
  HandleMap handles_;

  rclcpp::Time controllers_stamp_{ 0, 0, RCL_ROS_TIME };

  /**
   * @brief Protects access to managed_controllers_, active_controllers_, allocators_, handles_, and controllers_stamp.
   */
  std::mutex controllers_mutex_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_service_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_service_;

  // Chained controllers have dependent controllers (other controllers which must be started if the chained controller is started)
  std::unordered_map<std::string /* controller name */, std::vector<std::string> /* dependencies */> dependency_map_;
  // Controllers may have preceding chained controllers (other chained controllers which must be shutdown if the controller is shutdown)
  std::unordered_map<std::string /* controller name */, std::vector<std::string> /* reverse dependencies */>
      dependency_map_reverse_;

  /**
   * \brief Check if given controller is active
   * @param s state of controller
   * @return true if controller is active
   */
  static bool isActive(const controller_manager_msgs::msg::ControllerState& s)
  {
    return s.state == std::string("active");
  }

  /**
   * \brief  Call list_controllers and populate managed_controllers_ and active_controllers_. Allocates handles if
   * needed.
   * Throttled down to 1 Hz, controllers_mutex_ must be locked externally
   * @param force force rediscover
   */
  void discover(bool force = false)
  {
    // Skip if controller stamp is too new for new discovery, enforce update if force==true
    if (!force && ((node_->now() - controllers_stamp_) < CONTROLLER_INFORMATION_VALIDITY_AGE))
    {
      return;
    }

    controllers_stamp_ = node_->now();

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto result_future = list_controllers_service_->async_send_request(request);
    if (result_future.wait_for(service_call_timeout_) == std::future_status::timeout)
    {
      RCLCPP_WARN_STREAM(getLogger(), "Failed to read controllers from "
                                          << list_controllers_service_->get_service_name() << " within "
                                          << service_call_timeout_.count() << " seconds");
      return;
    }

    managed_controllers_.clear();
    active_controllers_.clear();

    auto result = result_future.get();
    if (!Ros2ControlManager::fixChainedControllers(result))
    {
      return;
    }

    for (const controller_manager_msgs::msg::ControllerState& controller : result->controller)
    {
      // If the controller is active, add it to the map of active controllers.
      if (isActive(controller))
      {
        // Get the names of the interfaces currently claimed by the active controller.
        auto& claimed_interfaces = active_controllers_.insert(std::make_pair(controller.name, controller))
                                       .first->second.claimed_interfaces;  // without namespace
        // Modify the claimed interface names in-place to only include the name of the joint and not the command type
        // (e.g. position, velocity, etc.).
        std::transform(claimed_interfaces.cbegin(), claimed_interfaces.cend(), claimed_interfaces.begin(),
                       [](const std::string& claimed_interface) {
                         return parseJointNameFromResource(claimed_interface);
                       });
      }

      // Instantiate a controller handle if one is available for this type of controller.
      if (loader_.isClassAvailable(controller.type))
      {
        std::string absname = getAbsName(controller.name);
        auto controller_it = managed_controllers_.insert(std::make_pair(absname, controller)).first;  // with namespace
        // Get the names of the interfaces that would be claimed by this currently-inactive controller if it was activated.
        auto& required_interfaces = controller_it->second.required_command_interfaces;
        // Modify the required interface names in-place to only include the name of the joint and not the command type
        // (e.g. position, velocity, etc.).
        std::transform(required_interfaces.cbegin(), required_interfaces.cend(), required_interfaces.begin(),
                       [](const std::string& required_interface) {
                         return parseJointNameFromResource(required_interface);
                       });
        allocate(absname, controller_it->second);
      }
    }
  }

  /**
   * \brief Allocates a MoveItControllerHandle instance for the given controller
   * Might create allocator object first.
   * @param name fully qualified name of the controller
   * @param controller controller information
   */
  void allocate(const std::string& name, const controller_manager_msgs::msg::ControllerState& controller)
  {
    if (handles_.find(name) == handles_.end())
    {
      const std::string& type = controller.type;
      AllocatorsMap::iterator alloc_it = allocators_.find(type);
      if (alloc_it == allocators_.end())
      {  // create allocator if needed
        alloc_it = allocators_.insert(std::make_pair(type, loader_.createUniqueInstance(type))).first;
      }

      std::vector<std::string> resources;
      // Collect claimed resources across different hardware interfaces
      for (const auto& resource : controller.claimed_interfaces)
      {
        resources.push_back(parseJointNameFromResource(resource));
      }

      moveit_controller_manager::MoveItControllerHandlePtr handle =
          alloc_it->second->alloc(node_, name, resources);  // allocate handle
      if (handle)
        handles_.insert(std::make_pair(name, handle));
    }
  }

  /**
   * \brief Get fully qualified name
   * @param name name to be resolved to an absolute name
   * @return resolved name
   */
  std::string getAbsName(const std::string& name)
  {
    return rclcpp::names::append(ns_, name);
  }

public:
  /**
   * \brief The default constructor. Reads the namespace from ~ros_control_namespace param and defaults to /
   */
  Ros2ControlManager()
    : loader_("moveit_ros_control_interface", "moveit_ros_control_interface::ControllerHandleAllocator")
  {
  }

  /**
   * \brief Configure interface with namespace
   * @param ns namespace of ros_control node (without /controller_manager/)
   */
  Ros2ControlManager(const std::string& ns)
    : ns_(ns), loader_("moveit_ros_control_interface", "moveit_ros_control_interface::ControllerHandleAllocator")
  {
    RCLCPP_INFO_STREAM(getLogger(), "Started moveit_ros_control_interface::Ros2ControlManager for namespace " << ns_);
  }

  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
    if (ns_.empty())
    {
      if (!node_->has_parameter("ros_control_namespace"))
      {
        ns_ = node_->declare_parameter<std::string>("ros_control_namespace", "/");
      }
      else
      {
        node_->get_parameter<std::string>("ros_control_namespace", ns_);
      }
    }

    double timeout_seconds;
    if (!node_->has_parameter("controller_service_call_timeout"))
    {
      timeout_seconds =
          node_->declare_parameter<double>("controller_service_call_timeout", DEFAULT_SERVICE_CALL_TIMEOUT);
    }
    else
    {
      node_->get_parameter("controller_service_call_timeout", timeout_seconds);
    }
    service_call_timeout_ = std::chrono::duration<double>(timeout_seconds);

    RCLCPP_INFO_STREAM(getLogger(), "Using service call timeout " << service_call_timeout_.count() << " seconds");

    list_controllers_service_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        getAbsName("controller_manager/list_controllers"));
    switch_controller_service_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        getAbsName("controller_manager/switch_controller"));

    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    discover(true);
  }
  /**
   * \brief Find and return the pre-allocated handle for the given controller.
   * @param name
   * @return
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    HandleMap::iterator it = handles_.find(name);
    if (it != handles_.end())
    {  // controller is manager by this interface
      return it->second;
    }
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /**
   * \brief Refresh controller list and output all managed controllers
   * @param[out] names list of controllers (with namespace)
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    discover();

    for (std::pair<const std::string, controller_manager_msgs::msg::ControllerState>& managed_controller :
         managed_controllers_)
    {
      names.push_back(managed_controller.first);
    }
  }

  /**
   * \brief Refresh controller list and output all active, managed controllers
   * @param[out] names list of controllers (with namespace)
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    discover();

    for (std::pair<const std::string, controller_manager_msgs::msg::ControllerState>& managed_controller :
         managed_controllers_)
    {
      if (isActive(managed_controller.second))
        names.push_back(managed_controller.first);
    }
  }

  /**
   * \brief Read interface names required by each controller from the cached controller state info.
   * @param[in] name name of controller (with namespace)
   * @param[out] joints
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    ControllersMap::iterator it = managed_controllers_.find(name);
    if (it != managed_controllers_.end())
    {
      for (const auto& required_resource : it->second.required_command_interfaces)
      {
        joints.push_back(required_resource);
      }
    }
  }

  /**
   * \brief Refresh controller state and output the state of the given one, only active_ will be set
   * @param[in] name name of controller (with namespace)
   * @return state
   */
  ControllerState getControllerState(const std::string& name) override
  {
    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    discover();

    ControllerState c;
    ControllersMap::iterator it = managed_controllers_.find(name);
    if (it != managed_controllers_.end())
    {
      c.active_ = isActive(it->second);
    }
    return c;
  }

  /**
   * \brief Filter lists for managed controller and computes switching set.
   * Stopped list might be extended by unsupported controllers that claim needed resources
   * @param activate_base vector of controllers to be activated
   * @param deactivate_base vector of controllers to be deactivated
   * @return true if switching succeeded
   */
  bool switchControllers(const std::vector<std::string>& activate_base,
                         const std::vector<std::string>& deactivate_base) override
  {
    // add_all_dependencies traverses the provided dependency map and appends the results to the controllers vector.
    auto add_all_dependencies = [](const std::unordered_map<std::string, std::vector<std::string>>& dependencies,
                                   const std::function<bool(const std::string&)>& should_include,
                                   std::vector<std::string>& controllers) {
      auto queue = controllers;
      while (!queue.empty())
      {
        auto controller = queue.back();
        queue.pop_back();
        if (controller.size() > 1 && controller[0] == '/')
        {
          // Remove leading / from controller name
          controller.erase(0, 1);
        }
        if (dependencies.find(controller) == dependencies.end())
        {
          continue;
        }
        for (const auto& dependency : dependencies.at(controller))
        {
          queue.push_back(dependency);
          if (should_include(dependency))
          {
            controllers.push_back("/" + dependency);
          }
        }
      }
    };

    std::vector<std::string> activate = activate_base;
    add_all_dependencies(
        dependency_map_,
        [this](const std::string& dependency) {
          return active_controllers_.find(dependency) == active_controllers_.end();
        },
        activate);
    std::vector<std::string> deactivate = deactivate_base;
    add_all_dependencies(
        dependency_map_reverse_,
        [this](const std::string& dependency) {
          return active_controllers_.find(dependency) != active_controllers_.end();
        },
        deactivate);

    // The dependencies for chained controller activation must be started first, but they are processed last, so the
    // order needs to be flipped
    std::reverse(activate.begin(), activate.end());

    std::scoped_lock<std::mutex> lock(controllers_mutex_);
    discover(true);

    // Holds the list of controllers that are currently active and their resources
    // Example:
    // controller1:
    //  - controller1_joint1
    //  - controller1_joint2
    //  ...
    // controller2:
    //  - controller2_joint1
    //  - controller2_joint2
    //  ...
    // ...
    // The left type have to be an unordered_multiset_of, because each controller can claim multiple resources
    // {{ "controller1", "controller1_joint1" },
    //  { "controller1", "controller1_joint2" },
    //  ...,
    //  { "controller2", "controller2_joint1" },
    //  { "controller2", "controller2_joint2" },
    //  ...}
    typedef boost::bimap<boost::bimaps::unordered_multiset_of<std::string>, std::string> resources_bimap;

    resources_bimap claimed_resources;

    // fill bimap with active controllers and their resources
    for (std::pair<const std::string, controller_manager_msgs::msg::ControllerState>& active_controller :
         active_controllers_)
    {
      for (const auto& resource : active_controller.second.claimed_interfaces)
      {
        claimed_resources.insert(resources_bimap::value_type(active_controller.second.name, resource));
      }
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    for (const std::string& it : deactivate)
    {
      ControllersMap::iterator c = managed_controllers_.find(it);
      if (c != managed_controllers_.end())
      {  // controller belongs to this manager
        request->deactivate_controllers.push_back(c->second.name);
        claimed_resources.left.erase(c->second.name);  // remove resources
      }
    }

    // For each controller to activate, find conflicts between the interfaces required by that controller and the
    // interfaces claimed by currently-active controllers.
    for (const std::string& it : activate)
    {
      ControllersMap::iterator c = managed_controllers_.find(it);
      if (c != managed_controllers_.end())
      {  // controller belongs to this manager
        request->activate_controllers.push_back(c->second.name);
        for (const auto& required_resource : c->second.required_command_interfaces)
        {
          resources_bimap::right_const_iterator res = claimed_resources.right.find(required_resource);
          if (res != claimed_resources.right.end())
          {  // resource is claimed
            if (std::find(request->deactivate_controllers.begin(), request->deactivate_controllers.end(),
                          res->second) == request->deactivate_controllers.end())
            {
              request->deactivate_controllers.push_back(res->second);  // add claiming controller to stop list
            }
            claimed_resources.left.erase(res->second);  // remove claimed resources
          }
        }
      }
    }

    // ROS2 Control expects simplified controller activation/deactivation.
    // E.g. a controller should not be stopped and started at the same time, rather it should be removed from both the
    // activation and deactivation request.
    deconflictControllerActivationLists(request->activate_controllers, request->deactivate_controllers);

    // Setting level to STRICT means that the controller switch will only be committed if all controllers are
    // successfully activated or deactivated.
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    if (!request->activate_controllers.empty() || !request->deactivate_controllers.empty())
    {  // something to switch?
      auto result_future = switch_controller_service_->async_send_request(request);
      if (result_future.wait_for(service_call_timeout_) == std::future_status::timeout)
      {
        RCLCPP_ERROR_STREAM(getLogger(), "Couldn't switch controllers at "
                                             << switch_controller_service_->get_service_name() << " within "
                                             << service_call_timeout_.count() << " seconds");
        return false;
      }
      discover(true);
      return result_future.get()->ok;
    }
    return true;
  }
  /**
   * \brief fixChainedControllers modifies ListControllers service response if it contains chained controllers.
   * Since chained controllers cannot be written to directly, they are removed from the response and their interfaces
   * are propagated back to the first controller with a non-chained input
   */
  bool fixChainedControllers(std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response>& result)
  {
    std::unordered_map<std::string, size_t> controller_name_map;
    for (size_t i = 0; i < result->controller.size(); ++i)
    {
      controller_name_map[result->controller[i].name] = i;
    }

    dependency_map_.clear();
    dependency_map_reverse_.clear();
    for (auto& controller : result->controller)
    {
      if (controller.chain_connections.size() > 1)
      {
        RCLCPP_ERROR_STREAM(getLogger(),
                            "Controller with name %s chains to more than one controller. Chaining to more than "
                            "one controller is not supported.");
        return false;
      }
      for (const auto& chained_controller : controller.chain_connections)
      {
        auto ind = controller_name_map[chained_controller.name];
        dependency_map_[controller.name].push_back(chained_controller.name);
        dependency_map_reverse_[chained_controller.name].push_back(controller.name);
        std::copy(result->controller[ind].reference_interfaces.begin(),
                  result->controller[ind].reference_interfaces.end(),
                  std::back_inserter(controller.required_command_interfaces));
      }
    }

    return true;
  }
};
/**
 *  \brief Ros2ControlMultiManager discovers all running ros_control node and delegates member function to the
 * corresponding Ros2ControlManager instances
 */
class Ros2ControlMultiManager : public moveit_controller_manager::MoveItControllerManager
{
  typedef std::map<std::string, moveit_ros_control_interface::Ros2ControlManagerPtr> ControllerManagersMap;
  ControllerManagersMap controller_managers_;
  rclcpp::Time controller_managers_stamp_{ 0, 0, RCL_ROS_TIME };
  std::mutex controller_managers_mutex_;

  rclcpp::Node::SharedPtr node_;

  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
  }
  /**
   * \brief  Poll for services and filter all controller_manager/list_controllers instances
   * Throttled down to 1 Hz, controller_managers_mutex_ must be locked externally
   */
  void discover()
  {
    // Skip if last discovery is too new for discovery rate
    if ((node_->now() - controller_managers_stamp_) < CONTROLLER_INFORMATION_VALIDITY_AGE)
      return;

    controller_managers_stamp_ = node_->now();

    const std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();

    for (const auto& service : services)
    {
      const auto& service_name = service.first;
      std::size_t found = service_name.find("controller_manager/list_controllers");
      if (found != std::string::npos)
      {
        std::string ns = service_name.substr(0, found);
        if (controller_managers_.find(ns) == controller_managers_.end())
        {  // create Ros2ControlManager if it does not exist
          RCLCPP_INFO_STREAM(getLogger(), "Adding controller_manager interface for node at namespace " << ns);
          auto controller_manager = std::make_shared<moveit_ros_control_interface::Ros2ControlManager>(ns);
          controller_manager->initialize(node_);
          controller_managers_.insert(std::make_pair(ns, controller_manager));
        }
      }
    }
  }

  /**
   * \brief Get namespace (including leading and trailing slashes) from controller name
   * @param name
   * @return extracted namespace or / if none is found
   */
  static std::string getNamespace(const std::string& name)
  {
    size_t pos = name.find('/', 1);
    if (pos == std::string::npos)
      pos = 0;
    return name.substr(0, pos + 1);
  }

public:
  /**
   * \brief Find appropriate interface and delegate handle creation
   * @param name
   * @return handle
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::unique_lock<std::mutex> lock(controller_managers_mutex_);

    std::string ns = getNamespace(name);
    ControllerManagersMap::iterator it = controller_managers_.find(ns);
    if (it != controller_managers_.end())
    {
      return it->second->getControllerHandle(name);
    }
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /**
   * \brief Read all managed controllers from discovered interfaces
   * @param names
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    std::unique_lock<std::mutex> lock(controller_managers_mutex_);
    discover();

    for (std::pair<const std::string, moveit_ros_control_interface::Ros2ControlManagerPtr>& controller_manager :
         controller_managers_)
    {
      controller_manager.second->getControllersList(names);
    }
  }

  /**
   * \brief Read all active, managed controllers from discovered interfaces
   * @param names
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    std::unique_lock<std::mutex> lock(controller_managers_mutex_);
    discover();

    for (std::pair<const std::string, moveit_ros_control_interface::Ros2ControlManagerPtr>& controller_manager :
         controller_managers_)
    {
      controller_manager.second->getActiveControllers(names);
    }
  }

  /**
   * \brief Find appropriate interface and delegate joints query
   * @param name
   * @param joints
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::unique_lock<std::mutex> lock(controller_managers_mutex_);

    std::string ns = getNamespace(name);
    ControllerManagersMap::iterator it = controller_managers_.find(ns);
    if (it != controller_managers_.end())
    {
      it->second->getControllerJoints(name, joints);
    }
  }

  /**
   * \brief Find appropriate interface and delegate state query
   * @param name
   * @return
   */
  ControllerState getControllerState(const std::string& name) override
  {
    std::unique_lock<std::mutex> lock(controller_managers_mutex_);

    std::string ns = getNamespace(name);
    ControllerManagersMap::iterator it = controller_managers_.find(ns);
    if (it != controller_managers_.end())
    {
      return it->second->getControllerState(name);
    }
    return ControllerState();
  }

  /**
   * \brief delegates switch to all known interfaces. Stops on first failing switch.
   * @param activate vector of controllers to be activated
   * @param deactivate vector of controllers to be deactivated
   * @return true if switching succeeded
   */
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) override
  {
    std::unique_lock<std::mutex> lock(controller_managers_mutex_);

    for (std::pair<const std::string, moveit_ros_control_interface::Ros2ControlManagerPtr>& controller_manager :
         controller_managers_)
    {
      if (!controller_manager.second->switchControllers(activate, deactivate))
        return false;
    }
    return true;
  }
};

}  // namespace moveit_ros_control_interface

PLUGINLIB_EXPORT_CLASS(moveit_ros_control_interface::Ros2ControlManager,
                       moveit_controller_manager::MoveItControllerManager);

PLUGINLIB_EXPORT_CLASS(moveit_ros_control_interface::Ros2ControlMultiManager,
                       moveit_controller_manager::MoveItControllerManager);
