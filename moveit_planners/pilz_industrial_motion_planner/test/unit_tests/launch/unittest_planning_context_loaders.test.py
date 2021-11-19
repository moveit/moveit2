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
from common_parameters import load_moveit_config


@pytest.mark.rostest
def generate_test_description():

    # Load the context
    test_config = load_moveit_config()

    # run test
    unittest_planning_context_loaders = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_planning_context_loaders",
        name="unittest_planning_context_loaders",
        parameters=[
            test_config.robot_description,
            test_config.robot_description_semantic,
            test_config.robot_description_kinematics,
            test_config.robot_description_planning,
            test_config.planning_plugin,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_planning_context_loaders,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"unittest_planning_context_loaders": unittest_planning_context_loaders},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, unittest_planning_context_loaders):
        proc_info.assertWaitForShutdown(
            process=unittest_planning_context_loaders, timeout=4000.0
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_planning_context_loaders):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=unittest_planning_context_loaders
        )
