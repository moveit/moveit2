#include <class_loader/class_loader.hpp>
#include <stomp_moveit/stomp_moveit_planning_context.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>

namespace stomp_moveit
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("stomp_moveit");

using namespace planning_interface;

class StompPlannerManager : public PlannerManager
{
public:
  StompPlannerManager()
  {
  }
  ~StompPlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    robot_model_ = model;
    node_ = node;
    parameter_namespace_ = parameter_namespace;

    param_listener_ = std::make_shared<stomp_moveit::ParamListener>(node, parameter_namespace);

    return true;
  }

  std::string getDescription() const override
  {
    return "STOMP";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs = { "STOMP" };
  }

  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const MotionPlanRequest& req,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (!canServiceRequest(req))
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return nullptr;
    }

    PlanningContextPtr planning_context =
        std::make_shared<StompPlanningContext>("STOMP", req.group_name, param_listener_->get_params());
    planning_context->setPlanningScene(planning_scene);
    planning_context->setMotionPlanRequest(req);

    return planning_context;
  }

  bool canServiceRequest(const MotionPlanRequest& req) const override
  {
    if (req.goal_constraints.empty())
    {
      RCLCPP_ERROR(LOGGER, "Invalid goal constraints");
      return false;
    }

    if (req.group_name.empty() || !robot_model_->hasJointModelGroup(req.group_name))
    {
      RCLCPP_ERROR(LOGGER, "Invalid joint group '%s'", req.group_name.c_str());
      return false;
    }

    const auto& pc = req.path_constraints;
    if (!(pc.joint_constraints.empty() && pc.position_constraints.empty() && pc.orientation_constraints.empty() &&
          pc.visibility_constraints.empty()))
    {
      RCLCPP_WARN(LOGGER, "Ignoring path constraints - not implemented!");
    }

    if (!req.trajectory_constraints.constraints.empty())
    {
      RCLCPP_WARN(LOGGER, "Ignoring trajectory constraints - not implemented!");
    }

    if (!req.reference_trajectories.empty())
    {
      RCLCPP_WARN(LOGGER, "Ignoring reference trajectories - not implemented!");
    }

    return true;
  }

  void setPlannerConfigurations(const PlannerConfigurationMap& pcs) override
  {
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  std::shared_ptr<stomp_moveit::ParamListener> param_listener_;
};

}  // namespace stomp_moveit

CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompPlannerManager, planning_interface::PlannerManager)
