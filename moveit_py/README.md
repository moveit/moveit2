## Notes on Python Bindings Modifications

1. `planning_interface::MotionPlanResponse` does not have the `start_state` and `planner_id` attributes. I commented out the related property definitions with their setters and getters.

2. In `planning_scene.cpp`, the method `PlanningScene::setAllowedCollisionMatrix` was not implemented in `moveit_core::planning_scene`. I copied the implementation, but since full compilation would be required, I commented out the setter method and replaced it with a read-only version.

3. In `trajectory_tools`, newer versions include `applyTOTGTimeParameterization`, which is not available here. It applies the Time-Optimal Trajectory Generation; currently commented out, but we could copy the implementation if needed.

4. In `planning_scene_monitor.cpp` (part of `moveit_ros`, now in `moveit_py`), `process_attached_collision_object` and `process_collision_object` were defined but are not actual methods of `PlanningSceneMonitor`—only of `PlanningScene`. Consider implementing them in `PlanningSceneMonitor` as well.

5. In `moveit.cpp`, `moveit/utils/logger.hpp` was used, but it no longer exists. It seems only `rclcpp`'s logger is now used. I followed the logging pattern from other modules.

6. In trajectory_Execution_manager commentato il getter execution_duration_monitoring e i vari getters che non c'erano in trajectory_execution_manager.h/.cpp.

7. In planning_component.hpp rimossi tutti i riferimenti a MultiPipelinePlanRequestParameters e plan con if se single_plan_request param o multi plan rimosso e lasciato parameters come .hpp di moveit2.

8. TODOOOO: In plannin_component.hpp/.cpp plan() ritornava un planning_interface::MotionPlanResponse, però in humble ritornava un moveit_cpp::PlanningComponent::PlanSolution. Anche in humble c'era definito un planning_interface::MotionPlanResponse in planning_response.hpp ma dato che torna un planSOlution ora ritorno plan solution. Però, non mi sembra definito la classe python corrispondente, come la faccio? Dove la metto?

9. In initPlanRequestParameters viene fatto params.load(node, ns) però ns non era usato in humble, non c'era proprio, come verranno gestiti i namespace?

10.                :py:class::`moveit_py.controller_manager.ExecutionStatus`: The status of the execution. \\ TODO(@samu): verify the module return type
