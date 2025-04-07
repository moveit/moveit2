from abc import ABC, abstractmethod
from typing import Any, Optional, Type
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from control_msgs.msg import JointJog


class Policy(ABC, Node):
    """
    Abstract base class for a policy node in ROS 2, designed to handle sensor inputs and 
    publish control commands using a learned or scripted controller.
    
    This class is intended to be subclassed. Subclasses must implement the `forward()` method
    which receives synchronized sensor data and publishes commands.
    """

    logger: Any
    """
    The ROS 2 logger associated with this node. Used for structured logging.
    """

    param_listener: Any
    """
    Object used to listen for or load dynamic parameters.
    Could be generated via `generate_parameter_library` or similar.
    """

    params: Any
    """
    Parameters associated with the policy, typically loaded from configuration files.
    """

    activate_policy_service: Any
    """
    ROS 2 service handle (of type `SetBool`) that allows external nodes to activate or deactivate the policy.
    """

    _is_active: bool
    """
    Internal flag indicating whether the policy is currently active.
    """

    def __init__(self, params: Any, node_name: str) -> None: 
        """
        Initializes the policy node.

        Args:
            params: A parameter object holding configuration values.
            node_name: The name of the ROS 2 node to be initialized.
        """
        ...

    @property
    def is_active(self) -> bool: 
        """
        Property to get the activation state of the policy.

        Returns:
            bool: True if the policy is active; otherwise, False.
        """
        ...

    def activate_policy(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response: 
        """
        ROS2 service callback to activate or deactivate the policy.

        Args:
            request: A `SetBool.Request` indicating the desired activation state.
            response: A `SetBool.Response` object to return the service result.

        Returns:
            SetBool.Response: Indicates success and updated policy state.
        """
        ...

    def get_sensor_msg_type(self, msg_type: str) -> Type[Image]: 
        """
        Returns the ROS message class corresponding to a sensor message type.

        Args:
            msg_type (str): The name of the sensor message type. Currently supports:
                - "sensor_msgs/Image"

        Returns:
            Type: The corresponding message class (e.g., `sensor_msgs.msg.Image`).
        """
        ...

    def get_command_msg_type(self, msg_type: str) -> Type[PoseStamped | Twist | JointJog]: 
        """
        Returns the ROS 2 message class for the specified control command type.

        Args:
            msg_type (str): The name of the control message type. Supported:
                - "geometry_msgs/PoseStamped"
                - "geometry_msgs/Twist"
                - "control_msgs/JointJog"

        Returns:
            Type: The corresponding command message class.
        """
        ...

    def register_sensors(self) -> None: 
        """
        Register the topics to listen to for sensor data.
        Registers and synchronizes the sensor message filters based on the
        parameter configuration. 
        """
        ...

    def register_command(self) -> None: 
        """
        Register the topic to publish actions to.
        Sets up the ROS 2 publisher to publish control commands based on
        the desired command output type from parameters.
        """
        ...

    @abstractmethod
    def forward(self) -> None: 
        """
        Perform a forward pass of the policy.
        Abstract method that subclasses must implement.
        """
        ...
