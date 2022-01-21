import launch_testing
import pytest
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.util import KeepAliveProc
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


@pytest.mark.rostest
def generate_test_description():

    moveit_configs = MoveItConfigsBuilder("moveit_resources_fanuc").to_dict()
    test_param = (
        ParameterBuilder("moveit_kinematics")
        .yaml("config/fanuc-kdl-singular-test.yaml")
        .to_dict()
    )

    fanuc_kdl_singular = Node(
        package="moveit_kinematics",
        executable="test_kinematics_plugin",
        name="fanuc_kdl_singular",
        parameters=[
            moveit_configs,
            test_param,
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                fanuc_kdl_singular,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"fanuc_kdl_singular": fanuc_kdl_singular},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, fanuc_kdl_singular):
        proc_info.assertWaitForShutdown(process=fanuc_kdl_singular, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
