import rclpy
from dataclasses import dataclass
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from moveit.servo_client.teleop import TeleopDevice

@dataclass
class DualShockAxes:
    """
    Axis mappings for the DualShock4 controller.
    """

    LEFT_STICK_X: int = 0
    LEFT_STICK_Y: int = 1
    LEFT_TRIGGER: int = 2
    RIGHT_STICK_X: int = 3
    RIGHT_STICK_Y: int = 4
    RIGHT_TRIGGER: int = 5
    D_PAD_X: int = 6
    D_PAD_Y: int = 7

@dataclass
class DualShockButtons:
    """
    Button mappings for the DualShock4 controller.
    """

    X: int = 0
    O: int = 1
    TRIANGLE: int = 2
    SQUARE: int = 3
    L1: int = 4
    R1: int = 5
    L2: int = 6
    R2: int = 7
    SHARE: int = 8
    OPTIONS: int = 9
    HOME: int = 10
    LEFT_STICK_TRIGGER: int = 11
    RIGHT_STICK_TRIGGER: int = 12

@dataclass
class PS4DualShock:
    """
    A dataclass to store the configuration for a PS4 DualShock controller.
    Conforms to the sensor_msgs/Joy message structure.
    """

    Axes: DualShockAxes = DualShockAxes()
    Buttons: DualShockButtons = DualShockButtons()

class PS4DualShockTeleop(TeleopDevice):
    """
    Teleoperation functionalities for a PS4 DualShock controller.
    """

    def __init__(
        self,
        ee_frame_name: str,
        node_name: str = "ps4_dualshock_teleop",
        device_name: str = "ps4_dualshock",
        device_config: PS4DualShock = PS4DualShock(),
    ) -> None:
        """
        Initialize the PS4DualShockTeleop instance.

        Args:
            ee_frame_name: End effector frame name.
            node_name: Name of the ROS node (default: "ps4_dualshock_teleop").
            device_name: Name of the teleop device (default: "ps4_dualshock").
            device_config: Configuration of the PS4 DualShock device (default: PS4DualShock()).
        """
        ...
    def publish_command(self, data: Joy) -> None:
        """
        Publishes the teleop command based on the Joy message data.

        Args:
            data: The Joy message containing input from the PS4 DualShock controller.
        """
        ...
    def record(self) -> None:
        """
        Records trajectory data (not implemented).
        """
        ...
