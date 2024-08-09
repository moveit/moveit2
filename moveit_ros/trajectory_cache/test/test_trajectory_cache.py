# Copyright 2024 Intrinsic Innovation LLC.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: methylDragon

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_pytest.tools import process as process_tools
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

import launch_pytest

import pytest
import os

# This would have been a gtest, but we need a move_group instance, which
# requires some parameters loaded and a separate node started.


@pytest.fixture
def moveit_config():
    return (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .planning_pipelines("ompl", ["ompl"])
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )


# We need this so the move_group has something to interact with
@pytest.fixture
def robot_fixture(moveit_config):
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

    return [
        run_move_group_node,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        *load_controllers,
    ]


@pytest.fixture
def trajectory_cache_test_runner_node(moveit_config):
    return Node(
        package="moveit_ros_trajectory_cache",
        executable="test_trajectory_cache",
        name="test_trajectory_cache_node",
        output="screen",
        cached_output=True,
        parameters=[moveit_config.to_dict()],
    )


@launch_pytest.fixture
def launch_description(trajectory_cache_test_runner_node, robot_fixture):
    return LaunchDescription(
        robot_fixture
        + [trajectory_cache_test_runner_node, launch_pytest.actions.ReadyToTest()]
    )


def validate_stream(expected):
    def wrapped(output):
        assert (
            expected in output.splitlines()
        ), f"Did not get expected: {expected} in output:\n\n{output}"

    return wrapped


@pytest.mark.launch(fixture=launch_description)
def test_all_tests_pass(trajectory_cache_test_runner_node, launch_context):
    # Check for occurrences of [PASS] in output
    assert process_tools.wait_for_output_sync(
        launch_context,
        trajectory_cache_test_runner_node,
        lambda x: x.count("[PASS]") == 169,  # All test cases passed.
        timeout=30,
    )

    # Check no occurrences of [FAIL] in output
    assert not process_tools.wait_for_output_sync(
        launch_context,
        trajectory_cache_test_runner_node,
        lambda x: "[FAIL]" in x,
        timeout=10,
    )

    yield
