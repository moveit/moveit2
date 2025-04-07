from typing import List, Optional
import moveit_msgs.msg
from moveit.core import robot_model, robot_state
import numpy as np

class RobotTrajectory:
    """
    Maintains a sequence of waypoints and the durations between these waypoints.
    """

    def __init__(self, robot_model: robot_model.RobotModel) -> None:
        """
        Initialize an empty robot trajectory from a robot model.

        Args:
            robot_model: The robot model associated with this trajectory.
        """
        ...

    def __getitem__(self, idx: int) -> robot_state.RobotState:
        """
        Get the waypoint at the specified index in the trajectory.

        Args:
            idx: The index of the waypoint.

        Returns:
            The robot state corresponding to the waypoint.
        """
        ...

    def __iter__(self):
        """
        Iterate over the waypoints in the trajectory.

        Returns:
            Iterator over the waypoints.
        """
        ...
        

    def __len__(self) -> int:
        """
        Get the number of waypoints in the trajectory.

        Returns:
            The number of waypoints.
        """
        ...

    def __reverse__(self) -> None:
        """
        Reverse the trajectory.
        """
        ...

    @property
    def joint_model_group_name(self) -> str:
        """
        The name of the joint model group associated with this trajectory.
        """
        ...

    @joint_model_group_name.setter
    def joint_model_group_name(self, value: str) -> None:
        """
        The name of the joint model group that this trajectory is for.
        """
        ...

    @property
    def robot_model(self) -> robot_model.RobotModel:
        """
        The robot model associated with this trajectory.
        """
        ...

    @property
    def duration(self) -> float:
        """
        The total duration of the trajectory.
        """
        ...

    @property
    def average_segment_duration(self) -> float:
        """
        The average duration of the segments in the trajectory.
        """
        ...

    def unwind(self) -> None:
        """
        Unwind the trajectory.
        """
        ...

    def get_waypoint_durations(self) -> List[float]:
        """
        Get the durations from the previous waypoint in the trajectory.

        Returns:
            Durations from the previous of each waypoints in the trajectory.
        """
        ...

    def apply_totg_time_parameterization(
        self,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        path_tolerance: float = 0.1,
        resample_dt: float = 0.1,
        min_angle_change: float = 0.001,
    ) -> bool:
        """
        Adds time parameterization using the Time-Optimal Trajectory Generation (TOTG) algorithm.

        Args:
            velocity_scaling_factor: The velocity scaling factor.
            acceleration_scaling_factor: The acceleration scaling factor.
            path_tolerance: Path tolerance for time parameterization (default: 0.1).
            resample_dt: Time step for parameterization (default: 0.1).
            min_angle_change: Minimum angle change for parameterization (default: 0.001).

        Returns:
            True if the trajectory was successfully parameterized, otherwise False.
        """
        ...

    def apply_ruckig_smoothing(
        self,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        mitigate_overshoot: bool = False,
        overshoot_threshold: float = 0.01,
    ) -> bool:
        """
        Applies Ruckig smoothing to the trajectory.

        Args:
            velocity_scaling_factor: The velocity scaling factor.
            acceleration_scaling_factor: The acceleration scaling factor.
            mitigate_overshoot: Whether to mitigate overshoot during smoothing (default: False).
            overshoot_threshold: Maximum allowed overshoot during smoothing (default: 0.01).

        Returns:
            True if the trajectory was successfully smoothed, otherwise False.
        """
        ...

    def get_robot_trajectory_msg(self, joint_filter: Optional[List[str]] = None) -> moveit_msgs.msg.RobotTrajectory:
        """
        Get the trajectory as a ROS RobotTrajectory message.

        Args:
            joint_filter: List of joints to include in the message. If None, all joints are included.

        Returns:
            A ROS RobotTrajectory message.
        """
        ...

    def set_robot_trajectory_msg(
        self, robot_state: robot_state.RobotState, msg: moveit_msgs.msg.RobotTrajectory
    ) -> None:
        """
        Set the trajectory from a ROS RobotTrajectory message.

        Args:
            robot_state: The reference robot starting state.
            msg: A ROS RobotTrajectory message.
        """
        ...
