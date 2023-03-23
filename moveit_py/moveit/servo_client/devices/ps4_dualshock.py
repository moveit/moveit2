# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Peter David Fagan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Peter David Fagan
""" PS4 dualshock teleop device implementation. """

import rclpy
from multiprocessing import Process

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from dataclasses import dataclass
from moveit.servo_client.teleop import TeleopDevice


###############################################################################
# DualShock4 Device Mappings
###############################################################################


@dataclass
class DualShockAxes:
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
    A dataclass to store device config. This class follows the conventions of Joy message (http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Joy.html)
    """

    Axes: DualShockAxes = DualShockAxes()

    Buttons: DualShockButtons = DualShockButtons()


###############################################################################
# DualShock4 Device Teleop
###############################################################################


class PS4DualShockTeleop(TeleopDevice):
    "A class which encapsulates teleoperation functionalities for ps4 dualshock device."

    def __init__(
        self,
        ee_frame_name: str,
        node_name: str = "ps4_dualshock_teleop",
        device_name: str = "ps4_dualshock",
        device_config: PS4DualShock = PS4DualShock(),
    ):
        super().__init__(
            node_name=node_name,
            device_name=device_name,
            device_config=device_config,
            ee_frame_name=ee_frame_name,
        )
        self.logger = rclpy.logging.get_logger("ps4_dualshock_teleop")

    def publish_command(self, data):
        """
        Publishes the teleop command.
        """
        try:
            # convert joy data to twist command
            twist = TwistStamped()
            twist.twist.linear.z = data.axes[self.device_config.Axes.RIGHT_STICK_Y]
            twist.twist.linear.y = data.axes[self.device_config.Axes.RIGHT_STICK_X]

            lin_x_right = -0.5 * (data.axes[self.device_config.Axes.RIGHT_TRIGGER])
            lin_x_left = 0.5 * (data.axes[self.device_config.Axes.LEFT_TRIGGER])
            twist.twist.linear.x = lin_x_right + lin_x_left

            twist.twist.angular.y = data.axes[self.device_config.Axes.LEFT_STICK_Y]
            twist.twist.angular.x = data.axes[self.device_config.Axes.LEFT_STICK_X]

            roll_positive = data.buttons[self.device_config.Buttons.R1]
            roll_negative = -1 * data.buttons[self.device_config.Buttons.L1]
            twist.twist.angular.z = float(roll_positive + roll_negative)

            twist.header.frame_id = self.ee_frame_name
            twist.header.stamp = self.get_clock().now().to_msg()
            self.twist_publisher.publish(twist)
        except Exception as e:
            self.logger.info.error(e)
            print(e)

    def record():
        pass
