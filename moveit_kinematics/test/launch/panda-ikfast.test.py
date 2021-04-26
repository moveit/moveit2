import launch_testing
import os
import pytest
import unittest
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

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file(
        "moveit_resources_panda_description", "urdf/panda.urdf"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    test_param = load_yaml("moveit_kinematics", "config/panda-ikfast-test.yaml")

    private_params = {
        "seed": [-0.5, -0.5, 0.3, -2, 0.8, 1.8, 1.9],
        "consistency_limits": [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
        "num_fk_tests": 100,
        "num_ik_tests": 100,
    }

    unit_tests_poses = {
        "unit_test_poses": {
            "size": 6,
            "pose_0": {
                "pose": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_1": {
                "pose": [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_2": {
                "pose": [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_3": {
                "pose": [0.0, -0.1, 0.0, 0.0, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_4": {
                "pose": [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_5": {
                "pose": [0.0, 0.0, -0.1, 0.0, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_6": {
                "pose": [0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_7": {
                "pose": [0.0, 0.0, 0.0, -0.1, 0.0, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_8": {
                "pose": [0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_9": {
                "pose": [0.0, 0.0, 0.0, 0.0, -0.1, 0.0],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_10": {
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
            "pose_11": {
                "pose": [0.0, 0.0, 0.0, 0.0, 0.0, -0.1],
                "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "type": "relative",
            },
        }
    }

    panda_ikfast = Node(
        package="moveit_kinematics",
        executable="test_kinematics_plugin",
        name="panda_kdl",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            test_param,
            private_params,
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                panda_ikfast,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"panda_ikfast": panda_ikfast},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, panda_ikfast):
        proc_info.assertWaitForShutdown(process=panda_ikfast, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
