import os
import pytest
import launch
import launch_ros
import launch_testing.actions
import launch_testing.asserts
import unittest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


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
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_servo_test_description(*args, gtest_name: SomeSubstitutionsType):

    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Get parameters for the Pose Tracking node
    pose_tracking_yaml = load_yaml("moveit_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "moveit_servo", "config/panda_simulated_config_pose_tracking.yaml"
    )
    servo_params = {"moveit_servo": servo_yaml}

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "panda_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in ["panda_arm_controller", "joint_state_controller"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Component nodes for tf and Servo
    test_container = ComposableNodeContainer(
        name="test_pose_tracking_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"/child_frame_id": "panda_link0", "/frame_id": "world"}],
            ),
        ],
        output="screen",
    )

    pose_tracking_gtest = launch_ros.actions.Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), gtest_name]
        ),
        parameters=[
            robot_description,
            robot_description_semantic,
            pose_tracking_params,
            servo_params,
            kinematics_yaml,
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
            test_container,
            TimerAction(period=2.0, actions=[pose_tracking_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
        + load_controllers
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
