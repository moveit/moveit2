import launch_testing
import pytest
import unittest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.util import KeepAliveProc

import sys
import os

sys.path.append(os.path.dirname(__file__))
from common_parameters import load_moveit_config, load_yaml


@pytest.mark.rostest
def generate_test_description():

    # Load the context
    test_config = load_moveit_config()

    test_param = load_yaml(
        "pilz_industrial_motion_planner", "config/unittest_joint_limits_aggregator.yaml"
    )

    # run test
    unittest_joint_limits_aggregator = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_joint_limits_aggregator",
        name="unittest_joint_limits_aggregator",
        parameters=[
            test_config.robot_description,
            test_config.robot_description_semantic,
            test_config.robot_description_kinematics,
            test_config.robot_description_planning,
            test_param,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_joint_limits_aggregator,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"unittest_joint_limits_aggregator": unittest_joint_limits_aggregator},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, unittest_joint_limits_aggregator):
        proc_info.assertWaitForShutdown(
            process=unittest_joint_limits_aggregator, timeout=4000.0
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_joint_limits_aggregator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=unittest_joint_limits_aggregator
        )
