#include "multi_state_validation_service_capability.h"
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupMultiStateValidationService::MoveGroupMultiStateValidationService() : MoveGroupCapability("StateValidationService")
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
    const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Response>& res)
{
    res->valid = true;
    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    
    auto robot_state = req->robot_state;
    moveit::core::RobotState rs = ls->getCurrentState();
    moveit::core::robotStateMsgToRobotState(robot_state, rs);

    for(int i=0; i<req->joint_states.size(); i++){

        // Update robot state with next set of joint states
        rs.setVariableValues(req->joint_states[i]);
        
        // configure collision request
        collision_detection::CollisionRequest creq;
        creq.group_name = req->group_name;
        creq.cost = true;
        creq.contacts = true;
        creq.max_contacts = ls->getWorld()->size() + ls->getRobotModel()->getLinkModelsWithCollisionGeometry().size();
        creq.max_cost_sources = creq.max_contacts;
        creq.max_contacts *= creq.max_contacts;
        collision_detection::CollisionResult cres;
        
        // check collision
        ls->checkCollision(creq, cres, rs);
        
        // copy contacts if any
        if (cres.collision)
        {
            rclcpp::Time time_now = context_->moveit_cpp_->getNode()->get_clock()->now();
            res->contacts.reserve(cres.contact_count);
            res->valid = false;
            for (collision_detection::CollisionResult::ContactMap::const_iterator it = cres.contacts.begin();
            it != cres.contacts.end(); ++it)
            for (const collision_detection::Contact& contact : it->second)
            {
                res->contacts.resize(res->contacts.size() + 1);
                collision_detection::contactToMsg(contact, res->contacts.back());
                res->contacts.back().header.frame_id = ls->getPlanningFrame();
                res->contacts.back().header.stamp = time_now;
            }
        }
        
        // copy cost sources
        res->cost_sources.reserve(cres.cost_sources.size());
        for (const collision_detection::CostSource& cost_source : cres.cost_sources)
        {
            res->cost_sources.resize(res->cost_sources.size() + 1);
            collision_detection::costSourceToMsg(cost_source, res->cost_sources.back());
        }
        
        // evaluate constraints
        if (!moveit::core::isEmpty(req->constraints))
        {
            kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
            kset.add(req->constraints, ls->getTransforms());
            std::vector<kinematic_constraints::ConstraintEvaluationResult> kres;
            kinematic_constraints::ConstraintEvaluationResult total_result = kset.decide(rs, kres);
            if (!total_result.satisfied)
            res->valid = false;
            
            // copy constraint results
            res->constraint_result.resize(kres.size());
            for (std::size_t k = 0; k < kres.size(); ++k)
            {
                res->constraint_result[k].result = kres[k].satisfied;
                res->constraint_result[k].distance = kres[k].distance;
            }
        }

        // No need to check any more joint states after the first invalid one
        if(!res->valid){
            break;
        }
    }
        
    return true;
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::MoveGroupMultiStateValidationService, move_group::MoveGroupCapability)
