#include "multi_state_validation_service_capability.hpp"
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/utils/message_checks.hpp>
#include <moveit/collision_detection/collision_tools.hpp>
#include <moveit/move_group/capability_names.hpp>

namespace move_group
{
MoveGroupMultiStateValidationService::MoveGroupMultiStateValidationService()
{
}

void MoveGroupMultiStateValidationService::initialize()
{
  validity_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::GetMultiStateValidity>(
    MULTI_STATE_VALIDITY_SERVICE_NAME, [this](const std::shared_ptr<rmw_request_id_t>& request_header,
                                          const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Request>& req,
                                          const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Response>& res) {
        return computeService(request_header, req, res);
      });
}

bool MoveGroupMultiStateValidationService::computeService(
    const std::shared_ptr<rmw_request_id_t>& /* unused */,
    const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Request>& req,
    const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Response>& res){

    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    moveit::core::RobotState rs = ls->getCurrentState();
    moveit::core::robotStateMsgToRobotState(req->robot_state, rs);

    for(size_t i=0; i<req->joint_states.size(); ++i){

        // Update robot state with next set of joint states
        rs.setVariableValues(req->joint_states[i]);

        // Check validity of given joint state
        res->valid = isStateValid(ls, rs, req->group_name, req->constraints, res->contacts, res->cost_sources, res->constraint_result);

        // Break on first invalid joint state
        /*
            Not necessarily, depends on whether users want speed or a complete list of the states that were invalid.
            You could consider adding an "early stop" boolean in the service request?
        */
        if(!res->valid){
            break;
        }
    }
        
    return true;
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupMultiStateValidationService, move_group::MoveGroupCapability)
