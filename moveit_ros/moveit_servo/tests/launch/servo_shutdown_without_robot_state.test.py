import unittest

import launch
import launch_ros
import launch_testing
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_test_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/test_config_panda.yaml")
        .to_dict()
    }

    # No ros2_control and no joint states, so Servo keeps waiting for a robot state.
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
        # If SIGINT is ignored, launch sends SIGTERM and then SIGKILL,
        # which gives a non-zero exit code.
        sigterm_timeout="10",
        sigkill_timeout="5",
    )

    return launch.LaunchDescription(
        [
            servo_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "servo_node": servo_node,
    }


class TestServoWaitsForRobotState(unittest.TestCase):
    def test_servo_is_waiting(self, proc_output, servo_node):
        proc_output.assertWaitFor(
            "Waiting for planning scene monitor to receive robot state update",
            process=servo_node,
            timeout=60.0,
        )


@launch_testing.post_shutdown_test()
class TestServoExitsOnSigint(unittest.TestCase):
    def test_servo_exit_code(self, proc_info, servo_node):
        # Shutdown sends SIGINT. Servo must exit by itself instead of being killed.
        launch_testing.asserts.assertExitCodes(proc_info, process=servo_node)
