from rclpy.impl.rcutils_logger import RCUtilsLogger
from moveit.servo_client.teleop import TeleopDevice as TeleopDevice
from multiprocessing import Process as Process
from sensor_msgs.msg import Joy as Joy
from std_srvs.srv import Trigger as Trigger

class DualShockAxes:
    LEFT_STICK_X: int
    LEFT_STICK_Y: int
    LEFT_TRIGGER: int
    RIGHT_STICK_X: int
    RIGHT_STICK_Y: int
    RIGHT_TRIGGER: int
    D_PAD_X: int
    D_PAD_Y: int
    def __init__(
        self,
        LEFT_STICK_X,
        LEFT_STICK_Y,
        LEFT_TRIGGER,
        RIGHT_STICK_X,
        RIGHT_STICK_Y,
        RIGHT_TRIGGER,
        D_PAD_X,
        D_PAD_Y,
    ) -> None: ...

class DualShockButtons:
    X: int
    O: int
    TRIANGLE: int
    SQUARE: int
    L1: int
    R1: int
    L2: int
    R2: int
    SHARE: int
    OPTIONS: int
    HOME: int
    LEFT_STICK_TRIGGER: int
    RIGHT_STICK_TRIGGER: int
    def __init__(
        self,
        X,
        O,
        TRIANGLE,
        SQUARE,
        L1,
        R1,
        L2,
        R2,
        SHARE,
        OPTIONS,
        HOME,
        LEFT_STICK_TRIGGER,
        RIGHT_STICK_TRIGGER,
    ) -> None: ...

class PS4DualShock:
    Axes: DualShockAxes
    Buttons: DualShockButtons
    def __init__(self, Axes, Buttons) -> None: ...

class PS4DualShockTeleop(TeleopDevice):
    logger: RCUtilsLogger
    def __init__(
        self,
        ee_frame_name: str,
        node_name: str = ...,
        device_name: str = ...,
        device_config: PS4DualShock = ...,
    ) -> None: ...
    def publish_command(self, data) -> None: ...
    def record() -> None: ...
