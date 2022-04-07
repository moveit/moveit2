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

    moveit_configs = MoveItConfigsBuilder("moveit_resources_panda").to_dict()
    test_param = (
        ParameterBuilder("moveit_kinematics")
        .yaml("config/panda-lma-test.yaml")
        .to_dict()
    )

    panda_lma = Node(
        package="moveit_kinematics",
        executable="test_kinematics_plugin",
        name="panda_lma",
        parameters=[
            moveit_configs,
            test_param,
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                panda_lma,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"panda_lma": panda_lma},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, panda_lma):
        proc_info.assertWaitForShutdown(process=panda_lma, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
