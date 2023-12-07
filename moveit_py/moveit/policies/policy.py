# Software License Agreement (BSD License)
#
# Copyright (c) 2023, Peter David Fagan
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

"""
Definition of an abstract base class for policy deployment.
For now, this policy only supports moveit_servo command interfaces and Image sensors.
"""

from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.qos import QoSProfile

from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image

from std_srvs.srv import SetBool

import message_filters


class Policy(ABC, Node):
    """An abstract base class for deploying learnt policies."""

    def __init__(self, params, node_name="policy_node"):
        """Initialise the policy."""
        super().__init__(node_name)
        self.logger = self.get_logger()

        # parse parameters
        self.param_listener = params.ParamListener(self)
        self.params = self.param_listener.get_params()

        # set policy to inactive by default
        self._is_active = False
        self.activate_policy_service = self.create_service(
            SetBool,
            "activate_policy",
            self.activate_policy,
        )

        # register sensor topics
        self.register_sensors()

        # register servo command topic
        self.register_command()

    @property
    def is_active(self):
        """Returns True if the policy is active."""
        return self._is_active

    @is_active.setter
    def active(self, value):
        """Sets the policy to active state via Python API."""
        self._is_active = value

    def activate_policy(self, request, response):
        """Sets the policy to active state via ROS interface."""
        self._is_active = request.data
        return response

    def get_sensor_msg_type(self, msg_type):
        """Returns the ROS 2 message type for a given sensor type."""
        if msg_type == "sensor_msgs/Image":
            return Image
        else:
            raise ValueError(f"Sensor type {msg_type} not supported.")

    def get_command_msg_type(self, msg_type):
        """Returns the ROS 2 message type for a given command type."""
        if msg_type == "geometry_msgs/PoseStamped":
            return PoseStamped
        elif msg_type == "geometry_msgs/Twist":
            return Twist
        elif msg_type == "control_msgs/JointJog":
            return JointJog
        else:
            raise ValueError(f"Command type {msg_type} not supported.")

    def register_sensors(self):
        """Register the topics to listen to for sensor data."""
        self.sensor_subs = []
        # TODO: refactor this section
        # Related Issue: https://github.com/PickNikRobotics/generate_parameter_library/issues/155
        for sensor_idx in range(self.params.num_sensors):
            sensor_params = self.get_parameters_by_prefix(f"sensor{sensor_idx + 1}")
            self.sensor_subs.append(
                message_filters.Subscriber(
                    self,
                    self.get_sensor_msg_type(sensor_params["type"].value),
                    str(sensor_params["topic"].value),
                    qos_profile=QoSProfile(depth=sensor_params["qos"].value),
                )
            )

        # create a synchronizer for the sensor topics
        self.sensor_sync = message_filters.ApproximateTimeSynchronizer(
            self.sensor_subs,
            self.params.sensor_queue,
            self.params.sensor_slop,
        )

        # register model forward pass as callback
        self.sensor_sync.registerCallback(self.forward)

    def register_command(self):
        """Register the topic to publish actions to."""
        self.command_pub = self.create_publisher(
            self.get_command_msg_type(self.params.command.type),
            self.params.command.topic,
            QoSProfile(depth=self.params.command.qos),
        )

    @abstractmethod
    def forward():
        """Perform a forward pass of the policy."""
        pass
