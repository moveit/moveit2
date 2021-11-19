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
        "pilz_industrial_motion_planner",
        "config/unittest_trajectory_generator_lin.yaml",
    )

    testdata_file_name = {
        "testdata_file_name": get_package_share_directory(
            "pilz_industrial_motion_planner"
        )
        + "/test_data/prbt/testdata_sequence.xml"
    }

    # run test
    unittest_trajectory_generator_lin = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_trajectory_generator_lin",
        name="unittest_trajectory_generator_lin",
        parameters=[
            test_config.robot_description,
            test_config.robot_description_semantic,
            test_config.robot_description_kinematics,
            test_config.robot_description_planning,
            test_param,
            testdata_file_name,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_trajectory_generator_lin,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"unittest_trajectory_generator_lin": unittest_trajectory_generator_lin},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, unittest_trajectory_generator_lin):
        proc_info.assertWaitForShutdown(
            process=unittest_trajectory_generator_lin, timeout=4000.0
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_trajectory_generator_lin):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=unittest_trajectory_generator_lin
        )
