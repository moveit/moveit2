from typing import Optional, List, Dict
import numpy as np

class VariableBounds:
    """
    Defines the variable bounds for a joint model.
    """

    @property
    def min_position(self) -> float:
        """
        Minimum position value for the joint.
        """
        ...
    @property
    def max_position(self) -> float:
        """
        Maximum position value for the joint.
        """
        ...
    @property
    def position_bounded(self) -> bool:
        """
        Indicates if the position is bounded.
        """
        ...
    @property
    def min_velocity(self) -> float:
        """
        Minimum velocity value for the joint.
        """
        ...
    @property
    def max_velocity(self) -> float:
        """
        Maximum velocity value for the joint.
        """
        ...
    @property
    def velocity_bounded(self) -> bool:
        """
        Indicates if the velocity is bounded.
        """
        ...
    @property
    def min_acceleration(self) -> float:
        """
        Minimum acceleration value for the joint.
        """
        ...
    @property
    def max_acceleration(self) -> float:
        """
        Maximum acceleration value for the joint.
        """
        ...
    @property
    def acceleration_bounded(self) -> bool:
        """
        Indicates if the acceleration is bounded.
        """
        ...
    @property
    def min_jerk(self) -> float:
        """
        Minimum jerk value for the joint.
        """
        ...
    @property
    def max_jerk(self) -> float:
        """
        Maximum jerk value for the joint.
        """
        ...
    @property
    def jerk_bounded(self) -> bool:
        """
        Indicates if the jerk is bounded.
        """
        ...

class JointModelGroup:
    """
    Represents a group of joints that are part of a robot model.
    """

    @property
    def name(self) -> str:
        """
        The name of the joint model group.
        """
        ...
    @property
    def link_model_names(self) -> List[str]:
        """
        List of names of link models in the group.
        """
        ...
    @property
    def joint_model_names(self) -> List[str]:
        """
        List of names of joint models in the group.
        """
        ...
    @property
    def active_joint_model_names(self) -> List[str]:
        """
        List of names of active joint models in the group.
        """
        ...
    @property
    def active_joint_model_bounds(self) -> List[VariableBounds]:
        """
        List of active joint model bounds for the group.
        """
        ...
    @property
    def eef_name(self) -> str:
        """
        The name of the end effector associated with the group.
        """
        ...
    def satisfies_position_bounds(
        self, values: np.ndarray, margin: float = 0.0
    ) -> bool:
        """
        Check if the joint positions satisfy the position bounds.

        Args:
            values: Joint position values as a NumPy array.
            margin: Allowed margin for bounds.

        Returns:
            True if the joint positions satisfy the bounds, False otherwise.
        """
        ...

class RobotModel:
    """
    Represents a kinematic model of a robot.
    """

    def __init__(self, urdf_xml_path: str, srdf_xml_path: str) -> None:
        """
        Initialize a RobotModel instance using URDF and SRDF files.

        Args:
            urdf_xml_path: Path to the URDF XML file.
            srdf_xml_path: Path to the SRDF XML file.
        """
        ...
    @property
    def name(self) -> str:
        """
        The name of the robot model.
        """
        ...
    @property
    def model_frame(self) -> str:
        """
        The frame in which transforms for the model are computed.
        """
        ...
    @property
    def root_joint_name(self) -> str:
        """
        The name of the root joint in the robot model.
        """
        ...
    def get_model_info(self) -> str:
        """
        Get a formatted string with information about the robot model.

        Returns:
            A string with robot model details.
        """
        ...
    @property
    def joint_model_group_names(self) -> List[str]:
        """
        Names of the joint model groups in the robot model.
        """
        ...
    @property
    def joint_model_groups(self) -> List[JointModelGroup]:
        """
        List of joint model groups available in the robot model.
        """
        ...
    @property
    def end_effectors(self) -> List[JointModelGroup]:
        """
        Get the map between end effector names and the groups they correspond to.
        """
        ...
    def has_joint_model_group(self, joint_model_group_name: str) -> bool:
        """
        Check if a joint model group with the given name exists.

        Args:
            joint_model_group_name: The name of the joint model group.

        Returns:
            True if the joint model group exists, False otherwise.
        """
        ...
    def get_joint_model_group(self, joint_model_group_name: str) -> JointModelGroup:
        """
        Get a joint model group by its name.

        Args:
            joint_model_group_name: The name of the joint model group.

        Returns:
            The joint model group instance.
        """
        ...
