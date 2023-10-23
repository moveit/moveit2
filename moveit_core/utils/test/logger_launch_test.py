# Software License Agreement (BSD License)
#
#  Copyright (c) 2023, PickNik Robotics Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  # Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  # Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  # Neither the name of Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

# Author: Tyler Weaver

# Based on https://github.com/ros2/launch/blob/master/launch_testing/test/launch_testing/examples/good_proc_launch_test.py

import os
import unittest
from threading import Event
from threading import Thread

import launch
import launch_ros
import launch_testing
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

import pytest


@pytest.mark.launch_test
def generate_test_description():
    # dut: device under test
    dut_process = launch_ros.actions.Node(
        package="moveit_core",
        executable="logger_test_node",
        name="logger_test_node",
        output="screen",
    )

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    return launch.LaunchDescription(
        [
            dut_process,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"dut_process": dut_process}


class MakeTestNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)
        self.msg_event_object = Event()
        self.rosout_msgs = []

    def start_subscriber(self):
        # Create a subscriber
        self.subscription = self.create_subscription(
            Log, "rosout", self.subscriber_callback, 10
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def subscriber_callback(self, data):
        self.rosout_msgs.append(data)
        self.msg_event_object.set()


# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestFixture(unittest.TestCase):
    def test_stdout_print(self, proc_output):
        proc_output.assertWaitFor("Before", timeout=10, stream="stdout")
        proc_output.assertWaitFor("After", timeout=10, stream="stdout")
        proc_output.assertWaitFor("Child", timeout=10, stream="stdout")

    def test_rosout_msgs_published(self, proc_output):
        rclpy.init()
        try:
            node = MakeTestNode("test_node")
            node.start_subscriber()
            msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
            assert msgs_received_flag, "Did not receive msgs !"
        finally:
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
