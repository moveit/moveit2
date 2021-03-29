<img src="https://github.com/ros-planning/moveit.ros.org/blob/main/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

# MoveIt 2 Beta - OMPL Constrained Planning Demo
## Setup
Before running the demo add the following lines to `ompl_planning.yaml` in the `panda_config` package
bellow `panda_arm:`

```
enforce_constrained_state_space: true
projection_evaluator: joints(panda_joint1,panda_joint2)
```

## Running
This demo includes a launch configuration for running MoveGroup and a separate demo node.

- Note: This demo shows how to construct a variety of
constraints. See [util.cpp](https://github.com/ros-planning/moveit2/blob/main/moveit_core/kinematic_constraints/src/utils.cpp) for helper functions to automate constructing the constraint messages.

The MoveGroup setup can be started:
```
ros2 launch run_ompl_constrained_planning run_move_group.launch.py
```

This allows you to start planning and executing motions:
```
ros2 launch run_move_group run_move_group_interface.launch.py
```

## Details
### State space selection process
There are three options for state space selection.

1. Set `enforce_constrained_state_space = true` AND there must be path constraints in the planning request. This overrides all other settings and selects a `ConstrainedPlanningStateSpace` factory. If there are no path constraints in the planning request, this option is ignored, the constrained state space is only usefull for paths constraints. At the moment only a single position constraint is supported.

2. Set `enforce_joint_model_state_space = true` and option 1. is false, then this overrides the remaining settings and selects `JointModelStateSpace` factory. Some planning problems such as orientation path constraints are represented in `PoseModelStateSpace` and sampled via IK. However, consecutive IK solutions are not checked for proximity at the moment and sometimes happen to be flipped, leading to invalid trajectories. This workaround lets the user prevent this problem by forcing rejection sampling in `JointModelStateSpace`.

3. When options 1. and 2. are both set to false, the factory is selected based on the priority each one returns. See [PoseModelStateSpaceFactory::canRepresentProblem](https://github.com/ros-planning/moveit2/blob/b4ff391133c2809e9f697d44593c89a77d1d4c5c/moveit_planners/ompl/ompl_interface/src/parameterization/work_space/pose_model_state_space_factory.cpp#L45) for details on the selection process. In short, it selects `PoseModelStateSpace` if there is an IK solver and a path constraint.
