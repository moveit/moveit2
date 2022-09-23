#pragma once

#include <moveit/planning_interface/planning_interface.h>

namespace stomp_moveit
{
class StompPlanningContext : public planning_interface::PlanningContext
{
public:
  using planning_interface::PlanningContext::PlanningContext;

  bool solve(planning_interface::MotionPlanResponse& res) override;

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  void clear() override;
};
}  // namespace stomp_moveit
