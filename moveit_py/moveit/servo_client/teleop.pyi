import abc
from abc import ABC, abstractmethod
from rclpy.node import Node
from typing import Any

class TeleopDevice(ABC, Node, metaclass=abc.ABCMeta):
    device_name: Any
    device_config: Any
    joy_subscriber: Any
    twist_publisher: Any
    servo_node_start_client: Any
    servo_node_stop_client: Any
    teleop_process: Any
    def __init__(self, node_name, device_name, device_config) -> None: ...
    def start_teleop(self) -> None: ...
    def stop_teleop(self) -> None: ...
    @abstractmethod
    def publish_command(self): ...
    @abstractmethod
    def record(self): ...
