from typing import Optional
import moveit_msgs.msg
import robot_trajectory

class MotionPlanResponse:
    """
    Motion plan response, containing the trajectory, planning time, error code, and start state.
    """

    def __init__(self):
        """
        Motion plan response, containing the trajectory, planning time, error code, and start state.
        """
        ...
    @property
    def trajectory(self) -> Optional[robot_trajectory.RobotTrajectory]:
        """
        The trajectory associated with the motion plan response.

        Returns:
            The trajectory of the motion plan response.
        """
        ...
    @property
    def planning_time(self) -> float:
        """
        The time spent planning the trajectory.

        Returns:
            The planning time in seconds.
        """
        ...
    @property
    def error_code(self) -> moveit_msgs.msg.MoveItErrorCodes:
        """
        The error code associated with the motion plan response.

        Returns:
            The error code (moveit_msgs.msg.MoveItErrorCodes) indicating the success or failure of the motion plan.
        """
        ...
    @property
    def start_state(self) -> moveit_msgs.msg.RobotState:
        """
        The start state of the robot when the motion plan was requested.

        Returns:
            The robot's start state.
        """
        ...
    @property
    def planner_id(self) -> str:
        """
        The ID of the planner used for this motion plan response.

        Returns:
            The planner's ID.
        """
        ...
    def __bool__(self) -> bool:
        """
        Boolean operator to check if the motion plan response indicates success.

        Returns:
            True if the motion plan was successful, False otherwise.
        """
        ...
