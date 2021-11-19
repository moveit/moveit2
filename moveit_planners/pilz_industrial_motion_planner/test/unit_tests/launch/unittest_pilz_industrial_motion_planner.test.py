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

    planning_plugin = {
        "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner"
    }

    # run test
    unittest_pilz_industrial_motion_planner = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_pilz_industrial_motion_planner",
        name="unittest_pilz_industrial_motion_planner",
        parameters=[
            test_config.robot_description,
            test_config.robot_description_semantic,
            test_config.robot_description_kinematics,
            test_config.robot_description_planning,
            planning_plugin,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_pilz_industrial_motion_planner,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "unittest_pilz_industrial_motion_planner": unittest_pilz_industrial_motion_planner
        },
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(
        self, proc_info, unittest_pilz_industrial_motion_planner
    ):
        proc_info.assertWaitForShutdown(
            process=unittest_pilz_industrial_motion_planner, timeout=4000.0
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_pilz_industrial_motion_planner):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=unittest_pilz_industrial_motion_planner
        )
