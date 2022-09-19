import launch_testing
import pytest
import unittest

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
        "pilz_industrial_motion_planner", "config/unittest_trajectory_functions.yaml"
    )

    # run test
    unittest_trajectory_functions = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_trajectory_functions",
        name="unittest_trajectory_functions",
        parameters=[
            test_config.to_dict(),
            test_param,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_trajectory_functions,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"unittest_trajectory_functions": unittest_trajectory_functions},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, unittest_trajectory_functions):
        proc_info.assertWaitForShutdown(
            process=unittest_trajectory_functions, timeout=4000.0
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_trajectory_functions):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=unittest_trajectory_functions
        )
