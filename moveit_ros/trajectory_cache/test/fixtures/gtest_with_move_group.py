import os
import unittest
import launch_testing

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_testing.actions import ReadyToTest

from moveit_configs_utils import MoveItConfigsBuilder


def generate_test_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .planning_pipelines("ompl", ["ompl"])
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict()],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="log",
    )

    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    gtest_node = Node(
        executable=PathJoinSubstitution(
            [
                LaunchConfiguration("test_binary_dir"),
                LaunchConfiguration("test_executable"),
            ]
        ),
        parameters=[moveit_config.to_dict()],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            *load_controllers,
            TimerAction(period=1.0, actions=[run_move_group_node]),
            TimerAction(period=3.0, actions=[gtest_node]),
            ReadyToTest(),
        ]
    ), {
        "gtest_node": gtest_node,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, gtest_node):
        self.proc_info.assertWaitForShutdown(gtest_node, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, gtest_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=gtest_node,
            # Allow -11 since ros2_control or warehouse_ros sometimes segfaults
            # for unrelated reasons.
            allowable_exit_codes=[0, -11],
        )
