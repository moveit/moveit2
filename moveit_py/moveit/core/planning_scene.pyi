from typing import Optional, List
import numpy as np
import moveit_msgs.msg
from moveit.core import (
    robot_model,
    robot_state,
    robot_trajectory,
    collision_detection,
    transforms,
)

class PlanningScene:
    """
    Representation of the environment as seen by a planning instance.
    The environment geometry, robot geometry, and state are maintained.
    """

    def __init__(
        self,
        robot_model: robot_model.RobotModel,
        world: Optional[collision_detection.World] = None,
    ) -> None:
        """
        Initialize a new PlanningScene object.

        Args:
            robot_model: The robot model associated with this planning scene.
            world: The collision world for this scene. Defaults to an empty world.
        """
        ...
    # Properties
    @property
    def name(self) -> str:
        """
        The name of the planning scene.
        """
        ...
    @name.setter
    def name(self, value: str) -> None:
        """
        Sets the name of the planning scene
        """
        ...
    @property
    def robot_model(self) -> robot_model.RobotModel:
        """
        The robot model associated with this planning scene.
        """
        ...
    @property
    def planning_frame(self) -> str:
        """
        The frame in which planning is performed.
        """
        ...
    @property
    def current_state(self) -> robot_state.RobotState:
        """
        The current state of the robot.
        """
        ...
    @current_state.setter
    def current_state(self, state: robot_state.RobotState) -> None:
        """
        Sets the given state to the current state of the robot model
        """
        ...
    @property
    def planning_scene_message(self) -> moveit_msgs.msg.PlanningScene:
        """
        The planning scene message representation.
        """
        ...
    @property
    def transforms(self) -> transforms.Transforms:
        """
        The transform manager for the planning scene.
        """
        ...
    @property
    def allowed_collision_matrix(self) -> moveit_msgs.msg.AllowedCollisionMatrix:
        """
        The allowed collision matrix for the planning scene.
        """
        ...
    # Methods
    def __copy__(self) -> "PlanningScene":
        """
        Create a shallow copy of the PlanningScene.
        """
        ...
    def __deepcopy__(self, memo: dict) -> "PlanningScene":
        """
        Create a deep copy of the PlanningScene.
        """
        ...
    def knows_frame_transform(
        self, robot_state: robot_state.RobotState, frame_id: str
    ) -> bool:
        """
        Check if a transform to the frame ID is known.

        Args:
            robot_state: The robot state to check
            frame_id: The frame ID to check.

        Returns:
            True if the transform is known, otherwise False.
        """
        ...
    def get_frame_transform(self, frame_id: str) -> np.ndarray:
        """
        Get the transform corresponding to the frame ID.

        Args:
            frame_id: The frame ID to get the transform for.

        Returns:
            A 4x4 numpy array representing the transform matrix.
        """
        ...
    def process_planning_scene_world(
        self, msg: moveit_msgs.msg.PlanningSceneWorld
    ) -> None:
        """
        Process a planning scene world message.

        Args:
            msg: The planning scene world message.
        """
        ...
    def apply_collision_object(
        self,
        collision_object_msg: moveit_msgs.msg.CollisionObject,
        color_msg: Optional[moveit_msgs.msg.ObjectColor] = None,
    ) -> None:
        """
        Apply a collision object to the planning scene.

        Args:
            collision_object_msg: The collision object to apply.
            color_msg: Optional color for the collision object.
        """
        ...
    def set_object_color(
        self, object_id: str, color_msg: moveit_msgs.msg.ObjectColor
    ) -> None:
        """
        Set the color of a collision object.

        Args:
            object_id: The ID of the collision object.
            color_msg: The color message.
        """
        ...
    def process_attached_collision_object(
        self, object: moveit_msgs.msg.AttachedCollisionObject
    ) -> None:
        """
        Process an attached collision object.

        Args:
            object: The attached collision object.
        """
        ...
    def process_octomap(self, msg: moveit_msgs.msg.Octomap) -> None:
        """
        Process an Octomap message.

        Args:
            msg: The Octomap message.
        """
        ...
    def remove_all_collision_objects(self) -> None:
        """
        Remove all collision objects from the planning scene.
        """
        ...
    def is_state_valid(
        self,
        robot_state: robot_state.RobotState,
        joint_model_group_name: str,
        verbose: bool = False,
    ) -> bool:
        """
        Check if the robot state is valid.

        Args:
            robot_state: The robot state to check.
            joint_model_group_name: The name of the joint group to validate.
            verbose: Whether to print detailed information.

        Returns:
            True if the state is valid, otherwise False.
        """
        ...
    def is_state_colliding(
        self,
        robot_state: robot_state.RobotState,
        joint_model_group_name: str,
        verbose: bool = False,
    ) -> bool:
        """
        Check if the robot state is in collision.

            Args:
            robot_state: The robot state to check collision for.
            joint_model_group_name: The name of the group to check collision for.
            verbose: If true, print the link names of the links in collision.
        Returns:
            True if the robot state is in collision, false otherwise.
        """
        ...
    def is_state_constrained(
        self,
        state: robot_state.RobotState,
        constraints: moveit_msgs.msg.Constraints,
        verbose: bool = False,
    ) -> bool:
        """
        Check if the robot state fulfills the passed constraints

           Args:
               state: The robot state to check constraints for.
               constraints: The constraints to check.
               verbose: If true, print the information
           Returns:
               True if state is constrained otherwise false.
        """
        ...
    def is_path_valid(
        self,
        trajectory: robot_trajectory.RobotTrajectory,
        joint_model_group_name: str,
        verbose: bool = False,
        invalid_indices: Optional[List[int]] = None,
    ) -> bool:
        """
        Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility)

        Args:
            trajectory: The robot trajectory to check.
            joint_model_group_name: The joint group to validate against.
            verbose: Whether to print detailed information.
            invalid_indices: List to store indices of invalid states (if provided).

        Returns:
            True if the path is valid, otherwise False.
        """
        ...
    def check_collision(
        self,
        collision_request: collision_detection.CollisionRequest,
        collision_result: collision_detection.CollisionResult,
        state: Optional[robot_state.RobotState] = None,
        acm: Optional[moveit_msgs.msg.AllowedCollisionMatrix] = None,
    ) -> bool:
        """
        Check whether the current state is in collision, and if needed, updates the collision transforms of the current state before the computation.

        Args:
            collision_request: The collision request object.
            collision_result: The collision result object to store results.
            state: Optional robot state to check.
            acm: Optional allowed collision matrix.

        Returns:
            True if there are collisions, otherwise False.
        """
        ...
    def check_collision_unpadded(
        self,
        collision_request: collision_detection.CollisionRequest,
        collision_result: collision_detection.CollisionResult,
        state: Optional[robot_state.RobotState] = None,
        acm: Optional[moveit_msgs.msg.AllowedCollisionMatrix] = None,
    ) -> bool:
        """
          Check whether a specified state (\e robot_state) is in collision, with respect to a given
        allowed collision matrix (\e acm), but use a collision_detection::CollisionRobot instance that has no padding.

             Args:
              collision_request: The collision request object.
              collision_result: The collision result object to store results.
              state: Optional robot state to check.
              acm: Optional allowed collision matrix.

             Returns:
              True if state is in collision otherwise false.
        """
        ...
    def check_self_collision(
        self,
        collision_request: collision_detection.CollisionRequest,
        collision_result: collision_detection.CollisionResult,
        state: Optional[robot_state.RobotState] = None,
        acm: Optional[moveit_msgs.msg.AllowedCollisionMatrix] = None,
    ) -> bool:
        """
        Check for self-collisions in the planning scene.

        Args:
            collision_request: The collision request object.
            collision_result: The collision result object to store results.
            state: Optional robot state to check.
            acm: Optional allowed collision matrix.

        Returns:
            True if there are self-collisions, otherwise False.
        """
        ...
    def save_geometry_to_file(self, file_name_and_path: str) -> bool:
        """
        Save the CollisionObjects in the PlanningScene to a file

        Args:
               file_path_and_name: The file to save the CollisionObjects to.

        Returns:
               True if save to file was successful otherwise false.
        """
        ...
    def load_geometry_from_file(self, file_name_and_path: str) -> bool:
        """
        Load the CollisionObjects from a file to the PlanningScene

        Args:
               file_path_and_name: The file to load the CollisionObjects from.

        Returns:
               True if load from file was successful otherwise false.
        """
        ...
