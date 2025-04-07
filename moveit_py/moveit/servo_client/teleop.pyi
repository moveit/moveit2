from abc import ABC, abstractmethod
import threading
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from typing import Optional

class TeleopDevice(ABC, Node):
    """
    An abstract base class for a teleoperation device.

    This class provides methods for starting and stopping teleoperation and expects the methods
    `publish_command` and `record` to be implemented by inheriting classes.
    """

    def __init__(
        self, node_name: str, device_name: str, device_config: dict, ee_frame_name: str
    ) -> None:
        """
        Initialize the teleoperation device.

        Args:
            node_name: The name of the ROS node.
            device_name: The name of the teleop device.
            device_config: Configuration for the teleop device.
            ee_frame_name: The end effector frame name.
        """
        ...
    device_name: str
    """
    The name of the teleop device.
    """

    device_config: dict
    """
    The configuration settings for the teleop device.
    """

    ee_frame_name: str
    """
    The end effector frame name for the teleop device.
    """

    joy_subscriber: Joy
    """
    Subscription to the Joy topic.
    """

    twist_publisher: TwistStamped
    """
    Publisher for delta twist commands.
    """

    servo_node_start_client: Trigger
    """
    Client for starting the servo node.
    """

    servo_node_stop_client: Trigger
    """
    Client for stopping the servo node.
    """

    teleop_thread: Optional[threading.Thread]
    """
    Thread for running the teleop process.
    """

    def start_teleop(self) -> None:
        """
        Starts the servo client and teleoperation process.

        This method starts the Servo ROS node and creates a multithreaded executor
        to handle concurrent teleop processes like robot movement and collision avoidance.
        """
        ...
    def stop_teleop(self) -> None:
        """
        Stops the servo client and teleoperation process.

        This method sends a stop signal to the Servo ROS node and terminates the teleop thread.
        """
        ...
    @abstractmethod
    def publish_command(self) -> None:
        """
        Publishes the teleop command to the appropriate topic.

        This method must be implemented by inheriting classes to define how teleop commands
        are published.
        """
        ...
    @abstractmethod
    def record(self) -> None:
        """
        Records trajectory data.

        This method must be implemented by inheriting classes to define how trajectory data
        is recorded.
        """
        ...
