from typing import Optional, Dict
import numpy as np
import geometry_msgs.msg
import moveit_msgs.msg
from moveit.core import robot_model


class RobotState:
    """
    Representation of a robot's state. A state is a collection of variables (position, velocity, acceleration, and effort) associated with joints and links. Operations are allowed at the variable level, joint level, and joint group level.
    """
    
    def __init__(self, robot_model: robot_model.RobotModel) -> None:
        """
        Initialize the RobotState object with a robot model.

        Args:
            robot_model: The robot model associated with the RobotState.
        """
        ...
        
    def __copy__(self) -> "RobotState":
        """
        Create a shallow copy of the RobotState.

        Returns:
            A new RobotState object.
        """
        ...
        
    def __deepcopy__(self, memo: dict) -> "RobotState":
        """
        Create a deep copy of the RobotState.

        Returns:
            A new RobotState object.
        """
        ...
        
    # Properties
    @property
    def robot_model(self) -> "robot_model.RobotModel":
        """
        The robot model instance associated with this robot state.
        """
        ...
        
    @property
    def dirty(self) -> bool:
        """
        Whether the RobotState is marked as dirty and requires updating.
        """
        ...
    
    @property
    def state_tree(self) -> str:
        """
        Represents the state tree of the robot state.
        """
        ...

    @property
    def state_info(self) -> str:
        """
        Outputs the state information of the robot state.
        """
        ...

    @property
    def joint_positions(self) -> Dict[str, float]:
        """
        Dictionary of joint names and their corresponding positions.
        """
        ...
        
    @joint_positions.setter
    def joint_positions(self, joint_positions: Dict[str, float]) -> None:
        """
        Set joint positions.

        Args:
            positions: Dictionary of joint names and their corresponding positions.
        """
        ...
        
    @property
    def joint_velocities(self) -> Dict[str, float]:
        """
        Dictionary of joint names and their corresponding velocities.
        """
        ...
        
    @joint_velocities.setter
    def joint_velocities(self, joint_velocities: Dict[str, float]) -> None:
        """
        Set joint velocities.

        Args:
            velocities: Dictionary of joint names and their corresponding velocities.
        """
        ...
        
    @property
    def joint_accelerations(self) -> Dict[str, float]:
        """
        Dictionary of joint names and their corresponding accelerations.
        """
        ...
        
    @joint_accelerations.setter
    def joint_accelerations(self, joint_accelerations: Dict[str, float]) -> None:
        """
        Set joint accelerations.

        Args:
            accelerations: Dictionary of joint names and their corresponding accelerations.
        """
        ...
        
    @property
    def joint_efforts(self) -> Dict[str, float]:
        """
        Dictionary of joint names and their corresponding efforts.
        """
        ...
        
    @joint_efforts.setter
    def joint_efforts(self, joint_efforts: Dict[str, float]) -> None:
        """
        Set joint efforts.

        Args:
            efforts: Dictionary of joint names and their corresponding efforts.
        """
        ...
        
    # Methods
    def update(self, force: bool = False, category: str = "all") -> None:
        """
        Update the state transforms.

        Args:
            force: when true forces the update of the transforms from scratch.
            category: specifies the category to update ("all", "links_only", "collisions_only").
            All indicates updating all transforms while "links_only" and "collisions_only" ensure that only links or collision transforms are updated.
        """
        ...
        
    def get_frame_transform(self, frame_id: str) -> np.ndarray:
        """
        Get the transformation matrix from the model frame (root of model) to the frame identified by frame_id.
        If frame_id was not found, frame_found is set to false and an identity transform is returned.
        This method is restricted to frames defined within the robot state and doesn't include collision object
        present in the collision world. Please use the PlanningScene.get_frame_transform method for collision world objects.

        Args:
            frame_id: The frame ID to query.

        Returns:
            A 4x4 transformation matrix between model frame and frame of frame_id.
        """
        ...
        
    def get_pose(self, link_name: str) -> geometry_msgs.msg.Pose:
        """
        Get the pose of the specified link in the robot mode.

        Args:
            link_name: The name of the link to query

        Returns:
            The pose of the link as a geometry_msgs.msg.Pose object.
        """
        ...
        
    def get_jacobian(
        self,
        joint_model_group_name: str,
        reference_point_position: np.ndarray,
        link_name: Optional[str] = None,
        use_quaternion_representation: bool = False,
    ) -> np.ndarray:
        """
        Compute the Jacobian matrix.

        Args:
            joint_model_group_name: The name of the joint model group.
            reference_point_position: A 3D vector for the reference point position.
            link_name: Optional link name to compute the Jacobian for.
            use_quaternion_representation: Use quaternion representation for orientation.

        Returns:
            The Jacobian matrix as a NumPy array with respect to reference point.
        """
        ...
        
    def set_joint_group_positions(
        self, joint_model_group_name: str, position_values: np.ndarray
    ) -> None:
        """
        Set the positions for a joint model group.

        Args:
            joint_model_group_name: The name of the joint model group.
            position_values: A NumPy array of joint positions.
        """
        ...
    
    def set_joint_group_active_positions(
         self, joint_model_group_name: str, position_values: np.ndarray
    ) -> None:   
        """
        Sets the active positions of joints in the specified joint model group.

        Args:
            joint_model_group_name: The name of the joint model group to set the active positions for.
            position_values: The positions of the joints in the joint model group.
        """
        ...

    def get_joint_group_positions(self, joint_model_group_name: str) -> np.ndarray:
        """
        Get the positions for a joint model group.

        Args:
            joint_model_group_name: The name of the joint model group.

        Returns:
            A NumPy array of joint positions.
        """
        ...
    
    def set_joint_group_velocities(
        self, joint_model_group_name: str, velocity_values: np.ndarray
    ) -> None:
        """
        Set the joint velocities for a joint model group.

        Args:
            joint_model_group_name: The name of the joint model group.
            velocity_values: A NumPy array of joint velocities.
        """
        ...

    def get_joint_group_velocities(self, joint_model_group_name: str) -> np.ndarray:
        """
        Get the joint velocities for a joint model group.

        Args:
            joint_model_group_name: The name of the joint model group.

        Returns:
            A NumPy array of joint velocities.
        """
        ...

    def set_joint_group_accelerations(
        self, joint_model_group_name: str, acceleration_values: np.ndarray
    ) -> None:
        """
        Sets the accelerations of the joints in the specified joint model group.

        Args:
            joint_model_group_name: The name of the joint model group.
            acceleration_values: A NumPy array of the accelerations of the joints in the joint model group.
        """
        ...

    def get_joint_group_accelerations(self, joint_model_group_name: str) -> np.ndarray:
        """
        For a given group, get the acceleration values of the variables that make up the group.

	   Args:
            joint_model_group_name: The name of the joint model group to copy the accelerations for.

	   Returns:
            The accelerations of the joints in the joint model group.
  
        """
        ...

    def get_global_link_transform(
            self, link_name:str
    ) -> np.ndarray:
        """
        Returns the transform of the specified link in the global frame.

       Args:
           link_name: The name of the link to get the transform for.

       Returns:
            The transform of the specified link in the global frame.
     
        """
        ...
    
    def set_from_ik(
            self,
            joint_model_group_name: str,
            geometry_pose: geometry_msgs.msg.Pose,
            tip_name: str,
            timeout: float
    ) -> None:
        """
        Sets the state of the robot to the one that results from solving the inverse kinematics for the specified group.

	   Args:
               joint_model_group_name: The name of the joint model group to set the state for.
               geometry_pose: The pose of the end-effector to solve the inverse kinematics for.
               tip_name: The name of the link that is the tip of the end-effector.
               timeout: The amount of time to wait for the IK solution to be found.
        """
        ...

    def set_to_default_values(
        self, joint_model_group_name: Optional[str] = None, state_name: Optional[str] = None
    ) -> bool:
        """
        Set joint positions to default values.
        The default position is 0, or if that is not within bounds then half way between min and max bound.

        Args:
            joint_model_group_name: The joint model group to modify.
            state_name: The predefined state name in the SRDF.

        Returns:
            True if the operation is successful, False otherwise.
        """
        ...
        
    def set_to_random_positions(
        self, joint_model_group_name: Optional[str] = None
    ) -> None:
        """
        Set joint positions to random values.

        Args:
            joint_model_group_name: The joint model group to modify.
        """
        ...
        
    def clear_attached_bodies(self) -> None:
        """
        Remove all attached collision bodies from the robot state.
        We only allow for attaching of objects via the PlanningScene instance.
        This method allows any attached objects that are associated to this RobotState instance to be removed
        """
        ...
        
def robotStateToRobotStateMsg(
        state: RobotState,
        copy_attached_bodies: bool = True
) -> moveit_msgs.msg.RobotState:
    """
    Converts the given robot state to an instance of RobotState msg from moveit_msgs
    """
    ...
