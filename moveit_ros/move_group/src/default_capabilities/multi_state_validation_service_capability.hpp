#pragma once

#include "state_validation_service_capability.hpp"
#include <moveit_msgs/srv/get_multi_state_validity.hpp>

namespace move_group
{
class MoveGroupMultiStateValidationService : public MoveGroupStateValidationService
{
public:
  void initialize() override;

private:
  bool computeService(const std::shared_ptr<rmw_request_id_t>& request_header,
                      const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Request>& req,
                      const std::shared_ptr<moveit_msgs::srv::GetMultiStateValidity::Response>& res);

  rclcpp::Service<moveit_msgs::srv::GetMultiStateValidity>::SharedPtr validity_service_;
};
}  // namespace move_group