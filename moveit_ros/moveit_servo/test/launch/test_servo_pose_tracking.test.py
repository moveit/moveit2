import os
import pytest
import launch
import launch_ros
import launch_testing
import unittest
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_servo_test_description(
    *args, gtest_name: launch.some_substitutions_type.SomeSubstitutionsType
):
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Pose Tracking and Servo nodes
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/panda_simulated_config_pose_tracking.yaml")
        .to_dict()
    }

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # Component nodes for tf and Servo
    test_container = launch_ros.actions.ComposableNodeContainer(
        name="test_pose_tracking_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"/child_frame_id": "panda_link0", "/frame_id": "world"}],
            ),
        ],
        output="screen",
    )

    pose_tracking_gtest = launch_ros.actions.Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [launch.substitutions.LaunchConfiguration("test_binary_dir"), gtest_name]
        ),
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            test_container,
            launch.actions.TimerAction(period=2.0, actions=[pose_tracking_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "test_container": test_container,
        "servo_gtest": pose_tracking_gtest,
        "ros2_control_node": ros2_control_node,
    }


def generate_test_description():
    return generate_servo_test_description(gtest_name="test_servo_pose_tracking")


class TestGTestProcessActive(unittest.TestCase):
    def test_gtest_run_complete(self, servo_gtest):
        self.proc_info.assertWaitForShutdown(servo_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(
        self, proc_info, test_container, servo_gtest, ros2_control_node
    ):
        launch_testing.asserts.assertExitCodes(proc_info, process=servo_gtest)
