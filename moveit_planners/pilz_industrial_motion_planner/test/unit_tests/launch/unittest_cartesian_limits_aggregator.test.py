import launch_testing
import os
import pytest
import unittest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.util import KeepAliveProc


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.full_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


@pytest.mark.rostest
def generate_test_description():
    cartesian_limits_yaml = load_yaml(
        "pilz_industrial_motion_planner",
        "config/unittest_cartesian_limits_aggregator.yaml",
    )

    # run test
    unittest_cartesian_limits_aggregator = Node(
        package="pilz_industrial_motion_planner",
        executable="unittest_cartesian_limits_aggregator",
        name="unittest_cartesian_limits_aggregator",
        parameters=[
            cartesian_limits_yaml,
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                unittest_cartesian_limits_aggregator,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"unittest_cartesian_limits_aggregator": unittest_cartesian_limits_aggregator},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, unittest_cartesian_limits_aggregator):
        proc_info.assertWaitForShutdown(
            process=unittest_cartesian_limits_aggregator, timeout=4000.0
        )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info, unittest_cartesian_limits_aggregator):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=unittest_cartesian_limits_aggregator
        )
