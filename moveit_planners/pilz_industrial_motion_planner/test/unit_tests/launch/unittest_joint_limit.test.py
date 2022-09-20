import launch_testing
import os
import sys
import pytest
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.util import KeepAliveProc

sys.path.append(os.path.dirname(__file__))
from common_parameters import load_yaml


@pytest.mark.rostest
def generate_test_description():
    joint_limits_yaml = load_yaml(
        "pilz_industrial_motion_planner", "config/unittest_joint_limit_config.yaml"
    )

    # run test
    unittest_joint_limit = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_joint_limit",
        name="unittest_joint_limit",
        parameters=[
            joint_limits_yaml,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_joint_limit,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"unittest_joint_limit": unittest_joint_limit},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, unittest_joint_limit):
        proc_info.assertWaitForShutdown(process=unittest_joint_limit, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_joint_limit):
        launch_testing.asserts.assertExitCodes(proc_info, process=unittest_joint_limit)
