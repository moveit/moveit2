# Simply check if the components stay alive after launching
import os
import sys
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import generate_common_hybrid_launch_description


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path_to_test = os.path.dirname(__file__)

    # Add the usual demo nodes
    ld = launch.LaunchDescription(generate_common_hybrid_launch_description())
    # Python testing requires this marker
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld


class TestFixture(unittest.TestCase):
    def test_startup(self, proc_output):
        rclpy.init()

        # Wait several seconds, allowing any nodes to die
        time.sleep(2)

        expected_nodes = [
            "controller_manager",
            "global_planner",
            "hybrid_planning_container",
            "hybrid_planning_demo_node",
            "joint_state_broadcaster",
            "local_planner",
            "robot_state_publisher",
        ]

        node = MakeTestNode("test_node")

        try:
            for node_name in expected_nodes:
                assert node.wait_for_node(node_name, 1.0), (
                    "Expected hybrid_planning node not found! Missing node: "
                    + node_name
                )
        finally:
            rclpy.shutdown()


class MakeTestNode(Node):
    def __init__(self, name="test_node"):
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print("Waiting for node...")
        while time.time() - start < timeout and not flag:
            flag = node_name in self.get_node_names()
            time.sleep(0.1)

        return flag
