#include <moveit/py_bindings_tools/gil_releaser.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/robot_interface/robot_interface.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>

namespace py = pybind11;
using moveit::py_bindings_tools::GILReleaser;

namespace moveit
{
namespace planning_interface
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("python_robot_interface");

class RobotInterfacePython : public RobotInterface
{
public:
  RobotInterfacePython(const std::string& move_group_node_name, const std::string& robot_description)
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("ROS does not seem to be running");
    }

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.arguments(
        { "--ros-args", "-r",
          "__node:=" + std::string("robot_interface_") + std::to_string(reinterpret_cast<std::size_t>(this)) });
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("_", options);

    // need to load robot_description, robot_description_semantic and kinematics_solvers parameters for robot_model_loader
    auto param_source = std::make_shared<rclcpp::SyncParametersClient>(node, move_group_node_name);
    while (!param_source->service_is_ready()) {};
    const std::string robot_description_semantic = robot_description + "_semantic";
    std::vector<std::string> interface_params = { robot_description, robot_description_semantic };
    rcl_interfaces::msg::ListParametersResult result =
        param_source->list_parameters({ "robot_description_kinematics" }, 3);
    auto kinematics_keys = result.names;
    for (auto key : kinematics_keys)
    {
      interface_params.push_back(key);
    }

    auto parameters = param_source->get_parameters(interface_params);
    node->set_parameters(parameters);

    private_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    private_executor_->add_node(node);
    // start executor on a different thread now
    private_executor_thread_ = std::thread([this]() { private_executor_->spin(); });
    new (this) RobotInterfacePython(node, robot_description);
  }

  RobotInterfacePython(const rclcpp::Node::SharedPtr& node, const std::string& robot_description)
    : RobotInterface(node, robot_description)
  {
  }

  ~RobotInterfacePython()
  {
    if (private_executor_thread_.joinable())
      private_executor_thread_.join();
    private_executor_.reset();
  }

  py::list getActiveJointNames() const
  {
    return py::cast(RobotInterface::getActiveJointNames());
  }

  py::list getGroupActiveJointNames(const std::string& group) const
  {
    return py::cast(RobotInterface::getGroupActiveJointNames(group));
  }

  py::list getJointNames() const
  {
    return py::cast(RobotInterface::getJointNames());
  }

  py::list getGroupJointNames(const std::string& group) const
  {
    return py::cast(RobotInterface::getGroupJointNames(group));
  }

  py::list getGroupJointTips(const std::string& group) const
  {
    return py::cast(RobotInterface::getGroupJointTips(group));
  }

  py::list getLinkNames() const
  {
    return py::cast(RobotInterface::getLinkNames());
  }

  py::list getGroupLinkNames(const std::string& group) const
  {
    return py::cast(RobotInterface::getGroupLinkNames(group));
  }

  py::list getGroupNames() const
  {
    return py::cast(RobotInterface::getGroupNames());
  }

  py::list getJointLimits(const std::string& name) const
  {
    return py::cast(RobotInterface::getJointLimits(name));
  }

  py::list getLinkPose(const std::string& name)
  {
    return py::cast(RobotInterface::getLinkPose(name));
  }

  py::list getDefaultStateNames(const std::string& group)
  {
    return py::cast(RobotInterface::getDefaultStateNames(group));
  }

  py::list getCurrentJointValues(const std::string& name)
  {
    GILReleaser gr;
    return py::cast(RobotInterface::getCurrentJointValues(name));
  }

  py::dict getJointValues(const std::string& group, const std::string& named_state)
  {
    return py::cast(RobotInterface::getJointValues(group, named_state));
  }

  py::dict getCurrentVariableValues()
  {
    GILReleaser gr;
    return py::cast(RobotInterface::getCurrentVariableValues());
  }

  py::tuple getEndEffectorParentGroup(const std::string& group)
  {
    return py::cast(RobotInterface::getEndEffectorParentGroup(group));
  }

  py::bytes getCurrentState()
  {
    GILReleaser gr;
    auto msg = RobotInterface::getCurrentState();
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkers()
  {
    GILReleaser gr;
    auto msg = RobotInterface::getRobotMarkers();
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkersPythonList(const std::vector<std::string>& link_names)
  {
    GILReleaser gr;
    auto msg = RobotInterface::getRobotMarkers(link_names);
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkersPythonDictList(const std::map<std::string, double>& values,
                                          const std::vector<std::string>& link_names)
  {
    GILReleaser gr;
    auto msg = RobotInterface::getRobotMarkers(values, link_names);
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkersPythonDict(const std::map<std::string, double>& values)
  {
    GILReleaser gr;
    auto msg = RobotInterface::getRobotMarkers(values);
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkersFromMsg(const py::bytes py_state_msg)
  {
    GILReleaser gr;
    moveit_msgs::msg::RobotState state_msg;
    py_bindings_tools::deserializeMsg(py_state_msg, state_msg);
    auto msg = RobotInterface::getRobotMarkers(state_msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkersGroup(const std::string& group)
  {
    GILReleaser gr;
    auto msg = RobotInterface::getRobotMarkersGroup(group);
    return py_bindings_tools::serializeMsg(msg);
  }

  py::bytes getRobotMarkersGroupPythonDict(const std::string& group, const std::map<std::string, double>& values)
  {
    GILReleaser gr;
    auto msg = RobotInterface::getRobotMarkersGroup(group, values);
    return py_bindings_tools::serializeMsg(msg);
  }

private:
  std::thread private_executor_thread_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> private_executor_;
};

}  // namespace planning_interface
}  // namespace moveit

using namespace moveit::planning_interface;

PYBIND11_MODULE(robot_interface, m)
{
  m.doc() = "MOVEIT2 robot interface.";
  py::class_<RobotInterfacePython>(m, "RobotInterface")
      .def(py::init<const std::string&, const std::string&>())
      .def("get_robot_name", &RobotInterfacePython::getRobotName)
      .def("get_active_joint_names", &RobotInterfacePython::getActiveJointNames)
      .def("get_group_active_joint_names", &RobotInterfacePython::getGroupActiveJointNames)
      .def("get_joint_names", &RobotInterfacePython::getJointNames)
      .def("get_group_joint_names", &RobotInterfacePython::getGroupJointNames)
      .def("get_group_joint_tips", &RobotInterfacePython::getGroupJointTips)
      .def("get_link_names", &RobotInterfacePython::getLinkNames)
      .def("get_group_link_names", &RobotInterfacePython::getGroupLinkNames)
      .def("get_group_names", &RobotInterfacePython::getGroupNames)
      .def("get_joint_limits", &RobotInterfacePython::getJointLimits)
      .def("get_planning_frame", &RobotInterfacePython::getPlanningFrame)
      .def("get_link_pose", &RobotInterfacePython::getLinkPose)
      .def("get_group_default_states", &RobotInterfacePython::getDefaultStateNames)
      .def("get_current_joint_values", &RobotInterfacePython::getCurrentJointValues)
      .def("get_joint_values", &RobotInterfacePython::getJointValues)
      .def("get_current_variable_values", &RobotInterfacePython::getCurrentVariableValues)
      .def("get_robot_root_link", &RobotInterfacePython::getRobotRootLink)
      .def("has_group", &RobotInterfacePython::hasGroup)
      .def("get_parent_group", &RobotInterfacePython::getEndEffectorParentGroup)
      .def("get_current_state", &RobotInterfacePython::getCurrentState)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkers)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonList)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersFromMsg)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDictList)
      .def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDict)
      .def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroup)
      .def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroupPythonDict);
}
