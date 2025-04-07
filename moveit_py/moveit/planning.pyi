# moveitpy.pyi

from typing import List, Optional, Any, Callable, ContextManager, Union
import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import numpy as np
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.core.robot_trajectory import RobotTrajectory

class MoveItPy:
    """
    The MoveItPy class is the main interface to the MoveIt Python API.
    """

    def __init__(
        self,
        node_name: str = "moveit_py",
        launch_params_filepaths: Optional[List[str]] = None,
        config_dict: Optional[dict] = None,
        provide_planning_service: bool = True,
    ) -> None:
        """
        Initialize MoveItPy instance and ROS node.

        Args:
            node_name: Name of the ROS node.
            launch_params_filepaths: File paths for ROS launch parameters.
            config_dict: Node configuration dictionary.
            provide_planning_service: Provide planning scene service if True.
        """
        ...
    def execute(
        self, robot_trajectory: RobotTrajectory, controllers: List[str]
    ) -> None:
        """
        Execute a robot trajectory.

        Args:
            robot_trajectory: The trajectory to execute.
            controllers: List of controllers to use.
        """
        ...
    def get_planning_component(
        self, planning_component_name: str
    ) -> "MoveItPy.PlanningComponent":
        """
        Get a planning component instance by name.

        Args:
            planning_component_name: Name of the planning component.

        Returns:
            A planning component instance corresponding to the provided plan component name.
        """
        ...
    def shutdown(self) -> None:
        """Shut down the ROS node."""
        ...
    def get_robot_model(self) -> RobotModel:
        """
        Get the RobotModel instance
        """
        ...
    def get_trajectory_execution_manager(self) -> "MoveItPy.TrajectoryExecutionManager":
        """
        Returns the trajectory execution manager.
        """
        ...
    def get_planning_scene_monitor(self) -> "MoveItPy.PlanningSceneMonitor":
        """
        Retrieve the PlanningSceneMonitor.

        Returns:
            PlanningSceneMonitor: The planning scene monitor instance.
        """
        ...

    class PlanningComponent:
        """
        Represents a planning component for motion planning.
        """

        def __init__(
            self, joint_model_group_name: str, moveit_py_instance: "MoveItPy"
        ) -> None:
            """
            Initialize a PlanningComponent.

            Args:
                joint_model_group_name: The name of the joint model group to plan for.
                moveit_py_instance: The MoveItPy instance.
            """
            ...
        def get_named_target_state_values(self, name: str) -> dict[str, float]:
            """
            dict: The joint values for targets specified by name.
            """
            ...
        @property
        def planning_group_name(self) -> str:
            """
            The name of the planning group to plan for.
            """
            ...
        @property
        def named_target_states(self) -> List[str]:
            """
            The names of the named robot states available as targets.
            """
            ...
        def set_start_state_to_current_state(self) -> bool:
            """
            Set the start state of the plan to the current state of the robot.
            """
            ...
        def set_start_state(
            self,
            configuration_name: str = None,
            robot_state: moveit_msgs.msg.RobotState = None,
        ) -> None:
            """
            Set the start state of the plan to the given robot state.

                Args:
                configuration_name: The name of the configuration to use as the start state.
                robot_state: The robot state to use as the start state.
            """
            ...
        def get_start_state(self) -> moveit_msgs.msg.RobotState:
            """
            Returns the current start state for the planning component.
            """
            ...
        def set_goal_state(
            self,
            configuration_name: Optional[str] = None,
            robot_state: Optional[moveit_msgs.msg.RobotState] = None,
            pose_stamped_msg: Optional[geometry_msgs.msg.PoseStamped] = None,
            pose_link: Optional[str] = None,
            motion_plan_constraints: Optional[List[moveit_msgs.msg.Constraints]] = None,
        ) -> bool:
            """
            Set the goal state for the planning component.

            Args:
                configuration_name: Name of the configuration.
                robot_state: Robot state as the goal.
                pose_stamped_msg: Goal pose message.
                pose_link: Link for pose constraint.
                motion_plan_constraints: Constraints for motion planning.

            Returns:
                True if goal set successfully, False otherwise.
            """
            ...
        def set_start_state(
            self,
            configuration_name: Optional[str] = None,
            robot_state: Optional[moveit_msgs.msg.RobotState] = None,
        ) -> bool:
            """
            Set the start state for the planning component.

            Args:
                configuration_name: Name of the configuration.
                robot_state: Robot state as the start state.

            Returns:
                True if start state set successfully, False otherwise.
            """
            ...
        def plan(
            self,
            single_plan_parameters: Optional[
                MoveItPy.PlanningComponent.PlanRequestParameters
            ] = None,
            multi_plan_parameters: Optional[
                MoveItPy.PlanningComponent.MultiPipelinePlanRequestParameters
            ] = None,
            planning_scene: Optional[object] = None,
            solution_selection_function: Optional[list[object]] = None,
            stopping_criterion_callback: Optional[list[bool]] = None,
        ) -> moveit_msgs.msg.RobotTrajectory:
            """
            Plan a motion plan using the current start and goal states.

            Args:
                single_plan_parameters: Parameters for single-pipeline planning.
                multi_plan_parameters: Parameters for multi-pipeline planning.
                planning_scene: The planning scene to use.
                solution_selection_function: A function for selecting solutions.
                stopping_criterion_callback: A callback for stopping criteria.

            Returns:
                The planned trajectory as a RobotTrajectory message.
            """
            ...
        def set_path_constraints(
            self, path_constraints: moveit_msgs.msg.Constraints
        ) -> None:
            """
            Set the path constraints for planning.

            Args:
                path_constraints: The path constraints to use.
            """
            ...
        def set_workspace(
            self,
            min_x: float,
            min_y: float,
            min_z: float,
            max_x: float,
            max_y: float,
            max_z: float,
        ) -> None:
            """
            Specify the workspace bounding box.

            Args:
                min_x: Minimum x value of the workspace.
                min_y: Minimum y value of the workspace.
                min_z: Minimum z value of the workspace.
                max_x: Maximum x value of the workspace.
                max_y: Maximum y value of the workspace.
                max_z: Maximum z value of the workspace.
            """
            ...
        def unset_workspace(self) -> None:
            """
            Remove the workspace bounding box from planning.
            """
            ...

        class PlanRequestParameters:
            """
            Planner parameters provided with a MotionPlanRequest.
            """

            def __init__(self, moveit_cpp, ns: str) -> None:
                """
                Planner parameters provided with a MotionPlanRequest.

                Args:
                    moveit_cpp: The MoveItCpp instance to use.
                    ns: The namespace in which the planner parameters will be loaded.
                """
                ...
            planner_id: str
            """The planner ID to use."""

            planning_pipeline: str
            """The planning pipeline to use."""

            planning_attempts: int
            """The number of planning attempts to make."""

            planning_time: float
            """The amount of time to spend planning."""

            max_velocity_scaling_factor: float
            """The maximum velocity scaling factor that can be used."""

            max_acceleration_scaling_factor: float
            """The maximum acceleration scaling factor that can be used."""

        class MultiPipelinePlanRequestParameters:
            """
            Planner parameters for multi-pipeline planning.
            """

            def __init__(self, planning_pipeline_names: List[str]) -> None:
                """
                Initialize the MultiPipelinePlanRequestParameters instance.

                Args:
                    planning_pipeline_names: A list of planning pipeline names to use in parallel.
                """
                ...
            @property
            def multi_plan_request_parameters(
                self,
            ) -> List[MoveItPy.PlanningComponent.PlanRequestParameters]: ...

    class PlanningSceneMonitor:
        """
        Maintains and monitors the internal state of the planning scene.
        """

        @property
        def name(self) -> str:
            """
            The name of this planning scene monitor.

            Returns:
                The name of the planning scene monitor.
            """
            ...
        def start_scene_monitor(self) -> None:
            """
            Starts the scene monitor.
            """
            ...
        def stop_scene_monitor(self) -> None:
            """
            Stops the scene monitor.
            """
            ...
        def start_state_monitor(self) -> None:
            """
            Starts the state monitor.
            """
            ...
        def stop_state_monitor(self) -> None:
            """
            Stops the state monitor.
            """
            ...
        def wait_for_current_robot_state(self) -> None:
            """
            Waits for the current robot state to be received.
            """
            ...
        def clear_octomap(self) -> None:
            """
            Clears the octomap in the planning scene.
            """
            ...
        def new_planning_scene_message(
            self, secen: moveit_msgs.msg.PlanningScene
        ) -> None:
            """
            Called to update the planning scene with a new message.

                Args:
               scene: The new planning scene message.
            """
            ...
        def process_attached_collision_object(
            self, attached_collision_objet_msg: moveit_msgs.msg.AttachedCollisionObject
        ) -> None:
            """
            Apply an attached collision object msg to the planning scene.

                Args:
               attached_collision_object_msg: The attached collision object to apply to the planning scene.
            """
            ...
        def process_collision_object(
            self,
            collision_object_msg: moveit_msgs.msg.CollisionObject,
            color_msg: moveit_msgs.msg.ObjectColor,
        ) -> None:
            """
            Apply a collision object to the planning scene.

               Args:
              collision_object_msg: The collision object to apply to the planning scene.
              color_msg: The color of the collision object to apply to the planning scene.
            """
            ...
        def request_planning_scene_state(self, service_name: str) -> None:
            """
             Request the planning scene from a service server.

            Args:
               service_name: The name of the service to call.
            """
            ...
        def update_frame_transforms(self) -> None:
            """
            Update the transforms for the frames that are not part of the kinematic model using tf.

            Examples of these frames are the "map" and "odom_combined" transforms.
            This function is automatically called when data that uses transforms is received.
            However, this function should also be called before starting a planning request, for example.
            """
            ...
        def read_only(self) -> LockedPlanningSceneContextManagerRO:
            """
            Returns a read-only context manager for the planning scene.

            Returns:
                LockedPlanningSceneContextManagerRO: The read-only context manager.
            """
            ...
        def read_write(self) -> LockedPlanningSceneContextManagerRW:
            """
            Returns a read-write context manager for the planning scene.

            Returns:
                LockedPlanningSceneContextManagerRW: The read-write context manager.
            """
            ...

        class LockedPlanningSceneContextManagerRO(ContextManager):
            """
            A context manager that locks the planning scene for reading.
            """

            def __enter__(self) -> "PlanningScene":
                """
                Provides access to a locked planning scene instance for read-only operations.

                Returns:
                    PlanningScene: The locked planning scene.
                """
                ...
            def __exit__(
                self,
                type: Optional[Any],
                value: Optional[Any],
                traceback: Optional[Any],
            ) -> None:
                """
                Releases the lock on the planning scene after exiting the context.

                Args:
                    type: The exception type, if any.
                    value: The exception value, if any.
                    traceback: The traceback object, if any.
                """
                ...

        class LockedPlanningSceneContextManagerRW(ContextManager):
            """
            A context manager that locks the planning scene for reading and writing.
            """

            def __enter__(self) -> "PlanningScene":
                """
                Provides access to a locked planning scene instance for read-write operations.

                Returns:
                    PlanningScene: The locked planning scene.
                """
                ...
            def __exit__(
                self,
                type: Optional[Any],
                value: Optional[Any],
                traceback: Optional[Any],
            ) -> None:
                """
                Releases the lock on the planning scene after exiting the context.

                Args:
                    type: The exception type, if any.
                    value: The exception value, if any.
                    traceback: The traceback object, if any.
                """
                ...

    class TrajectoryExecutionManager:
        """
        A class responsible for managing the execution of robot trajectories in MoveIt.

        It provides interfaces for pushing trajectories, monitoring execution, and handling
        controller state and feedback.
        """

        def __init__(self, robot_model: RobotModel, ns: str = "") -> None: ...
        def is_managing_controllers(self) -> bool:
            """
            If this function returns true, then this instance of the manager is allowed to load/unload/switch controllers.
            """
            ...
        def is_controller_active(self, controller: str) -> bool:
            """Check if a controller is active"""
            ...
        def are_controllers_active(self, controllers: list[str]) -> bool:
            """Checks is a set of controllers is active"""
            ...
        def process_event(self, event: str) -> None:
            """
            Execute a named event (e.g., 'stop').
            """
            ...
        def clear(self) -> None:
            """
            Clears all queued trajectories.
            """
            ...
        def push(
            self,
            trajectory: Union[
                moveit_msgs.msg.RobotTrajectory, trajectory_msgs.msg.JointTrajectory
            ],
            controller: str,
        ) -> None:
            """
            Add a trajectory for future execution. Optionally specify a controller to use for the trajectory.

            If no controller is specified, a default is used.

            Args:
                trajectory: The robot trajectory to be executed.Can be RobotTrajectory msg or JointTrajectory msg.
                controller: The controller to use for execution.
            """
            ...
        def push(
            self,
            trajectory: Union[
                moveit_msgs.msg.RobotTrajectory, trajectory_msgs.msg.JointTrajectory
            ],
            controllers: list[str],
        ) -> None:
            """
            Add a trajectory for future execution. Optionally specify a controller to use for the trajectory.

            Optionally specify a set of controllers to consider using for the trajectory.
            Multiple controllers can be used simultaneously to execute the different parts of the trajectory.
            If multiple controllers can be used, preference is given to the already loaded ones.
            If no controller is specified, a default is used.

            Args:
                trajectory: The robot trajectory to be executed.Can be RobotTrajectory msg or JointTrajectory msg.
                controllers: The set of controller to use for execution.
            """
            ...
        def execute(self, callback: Callable, auto_clear: bool = True) -> None:
            """
            Start the execution of pushed trajectories.

            This version does not wait for completion but calls a callback function once the full trajectory execution completes.

            Args:
                callback: A function to be called after trajectory execution completes. ExecutionCompleteCallback type.
                auto_clear: Whether to automatically clear the execution queue. Defaults to True.
            """
            ...
        def execute(
            self, callback: Callable, part_callback: Callable, auto_clear: bool = True
        ) -> None:
            """
            Start the execution of pushed trajectories with part-wise and full completion callbacks.

            This version does not wait for completion but supports:
              - a callback after the full trajectory finishes
              - a callback after each path segment completes

            Args:
                callback: Called once the full trajectory execution finishes. ExecutionCompleteCallback type.
                part_callback: Called after each trajectory segment completes successfully. PathSegmentCompleteCallback type.
                auto_clear: Whether to automatically clear the execution queue. Defaults to True.
            """
            ...
        def execute_and_wait(self, auto_clear: bool = True) -> None:
            """
            Execute the trajectory and wait until execution completes.

            Args:
                auto_clear: Whether to automatically clear the trajectory queue after execution. Defaults to True.
            """
            ...
        def wait_for_execution(self) -> None:
            """
            Waits until the current trajectory finishes execution.
            """
            ...
        def stop_execution(self, auto_clear: bool = True) -> None:
            """
            Stops active trajectory execution, if any immediately.

            Args:
                auto_clear: Whether to automatically clear the trajectory queue after stopping execution. Defaults to True.
            """
            ...
        def enable_execution_duration_monitoring(self, flag: bool) -> None:
            """
            Enable or disable monitoring of trajectory execution duration.

            When enabled, the manager checks if a controller exceeds the expected duration,
            and will cancel the trajectory if the execution time surpasses the threshold.

            Args:
                flag: True to enable monitoring, False to disable.
            """
            ...
        def execution_duration_monitoring(self) -> bool:
            """
            Returns whether execution duration monitoring is currently enabled.

            Returns:
                True if monitoring is enabled, False otherwise.
            """
        def get_last_execution_status(self) -> str:
            """
            Retrieves the status of the last trajectory execution.

            Returns:
                A string representing the execution status.
            """
            ...
        def ensure_active_controllers_for_group(self, group: str) -> bool:
            """
            Ensures that controllers required for the specified group are active.
            If the 'moveit_manage_controllers' parameter is false and the controllers that happen to be active do not cover the joints in the group to be actuated, this function fails.

            Args:
                group: The joint model group name.

            Returns:
                True if controllers are active and ready, False otherwise.
            """
            ...
        def ensure_active_controllers_for_joints(self, joints: str) -> bool:
            """
             Make sure the active controllers are such that trajectories that actuate joints in the specified set can be executed.

            If the 'moveit_manage_controllers' parameter is false and the controllers that happen to be active do not cover the joints to be actuated, this function fails.
            """
            ...
        def ensure_active_controller(self, controller: str) -> bool:
            """
            Ensure a particular controller is active.
            If the 'moveit_manage_controllers' parameter is false and the controllers that happen to be active do not include the one specified as argument, this function fails.
            """
            ...
        def ensure_active_controllers(self, controllers: list[str]) -> bool:
            """
            Make sure a particular set of controllers are active.
            If the 'moveit_manage_controllers' parameter is false and the controllers that happen to be active do not include the ones specified as argument, this function fails.
            """
            ...
        def set_allowed_execution_duration_scaling(self, scaling: float) -> None:
            """
            Sets the global scaling factor for maximum execution duration.
            """
            ...
        def allowed_execution_duration_scaling(self) -> float:
            """
            Get the current execution duration scaling factor.

            Returns:
                The multiplicative scaling factor used for allowed trajectory durations.
            """
            ...
        def set_allowed_goal_duration_margin(self, margin: float) -> None:
            """
            Sets the margin time allowed after the expected goal completion.
            """
            ...
        def allowed_goal_duration_margin(self) -> float:
            """
            Get the current margin of the duration of a trajectory to get the allowed duration of execution.

            Returns:
                The multiplicative scaling factor used for allowed goal durations.
            """
            ...
        def set_execution_velocity_scaling(self, scaling: float) -> None:
            """
            Sets the scaling factor for execution velocity.
            """
            ...
        def execution_velocity_scaling(self) -> float:
            """
            Get the current scaling of the execution velocities.

            Returns:
                The multiplicative scaling factor used for execution velocities.
            """
            ...
        def set_allowed_start_tolerance(self, tolerance: float) -> None:
            """
            Set the joint-value tolerance used to validate whether the start state of the planned
            trajectory matches the robot's current state.

            This helps prevent large discontinuities between actual and commanded positions
            at the beginning of trajectory execution.

            Args:
                tolerance: Maximum allowed deviation (in joint value) between
                                   the current and start state of the trajectory.
            """
            ...
        def allowed_start_tolerance(self) -> float:
            """
            Get the current joint-value tolerance for validating trajectory start conditions.

            Returns:
                Tolerance value (in joint units) used for validation.
            """
            ...
        def set_wait_for_trajectory_completion(self, flag: bool) -> None:
            """
            Enable or disable the behavior of waiting for the full trajectory execution to finish.

            If disabled, the execution returns control to the caller immediately after sending
            the trajectory to the controller, without waiting for its completion.

            Args:
                flag: True to wait for trajectory completion, False otherwise.
            """
            ...
        def wait_for_trajectory_completion(self) -> bool:
            """
            Check whether the system is currently set to wait for trajectory execution to complete.

            Returns:
                True if waiting for completion is enabled, False if it returns immediately.
            """
            ...
