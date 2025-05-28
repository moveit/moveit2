from typing import Optional, List
import moveit_msgs.msg
import moveit.core

def construct_link_constraint(
    link_name: str,
    source_frame: str,
    cartesian_position: Optional[List[float]] = None,
    cartesian_position_tolerance: Optional[float] = None,
    orientation: Optional[List[float]] = None,
    orientation_tolerance: Optional[float] = None,
) -> moveit_msgs.msg.Constraints:
    """
    Construct a link constraint message.

    Args:
        link_name: The name of the link for which the constraint is defined.
        source_frame: The source frame of reference.
        cartesian_position: The Cartesian position (x, y, z) to apply the constraint. Defaults to None.
        cartesian_position_tolerance: Tolerance for the Cartesian position. Defaults to None.
        orientation: The orientation (x, y, z, w) to apply the constraint. Defaults to None.
        orientation_tolerance: Tolerance for the orientation. Defaults to None.

    Raises:
        ValueError: If neither cartesian_position nor orientation is provided.

    Returns:
        The constructed Constraints message.
    """
    ...

def construct_joint_constraint(
    robot_state: moveit.core.RobotState,
    joint_model_group: moveit.core.JointModelGroup,
    tolerance: float = 0.01,
) -> moveit_msgs.msg.Constraints:
    """
    Construct a joint constraint message.

    Args:
        robot_state: The current state of the robot.
        joint_model_group: The joint model group for the robot.
        tolerance: The tolerance for the joint constraint. Defaults to 0.01.

    Returns:
        The constructed Constraints message.
    """
    ...

def construct_constraints_from_node(
    node_name: str, ns: str
) -> moveit_msgs.msg.Constraints:
    """
    Construct a constraint message from a node.

    Args:
        node_name: The ROS 2 node from which the constraints are constructed.
        ns: The namespace to use for the constraints.

    Returns:
        The constructed Constraints message.
    """
    ...
