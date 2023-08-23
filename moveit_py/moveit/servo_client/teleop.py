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
""" Definition of an abstract base class for device teleoperation. """

import rclpy

from abc import ABC, abstractmethod
import threading
from rclpy.node import Node

# messages/services
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class TeleopDevice(ABC, Node):
    """
    An abstract base class for a teleoperation device.

    This class expects both the `publish_command` and `record` methods to be overridden by inheriting classes.
    """

    def __init__(self, node_name, device_name, device_config, ee_frame_name):
        super().__init__(node_name=node_name)
        self.device_name = device_name
        self.device_config = device_config
        self.ee_frame_name = ee_frame_name

        # default subscriber and publisher setup
        self.joy_subscriber = self.create_subscription(
            Joy, "/joy", self.publish_command, 10
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.servo_node_start_client = self.create_client(
            Trigger, "/servo_node/start_servo"
        )
        self.servo_node_stop_client = self.create_client(
            Trigger, "/servo_node/stop_servo"
        )

        self.teleop_thread = None  # gets created when teleop starts

    def start_teleop(self):
        """
        Starts servo client and teleop process.
        """
        try:
            # start servo client
            self.servo_node_start_client.wait_for_service(10.0)
            self.servo_node_start_client.call_async(Trigger.Request())

            # spin the teleop node
            # use multithreaded because Servo has concurrent processes for moving the robot and avoiding collisions
            ex = rclpy.executors.MultiThreadedExecutor()
            ex.add_node(self)
            self.teleop_thread = threading.Thread(target=ex.spin, daemon=True)
            self.teleop_thread.start()
        except Exception as e:
            print(e)

    def stop_teleop(self):
        """
        Stops servo client and teleop process.
        """
        try:
            self.servo_node_stop_client.wait_for_service(10.0)
            self.servo_node_stop_client.call_async(Trigger.Request())
            self.teleop_thread.join()
        except Exception as e:
            print(e)

    @abstractmethod
    def publish_command(self):
        """
        Publishes the teleop command.
        """
        pass

    @abstractmethod
    def record(self):
        """
        Records trajectory data.
        """
        pass
