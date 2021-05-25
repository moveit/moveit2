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

    robot_description_config = load_file(
        "moveit_resources_fanuc_description", "urdf/fanuc.urdf"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "moveit_resources_fanuc_moveit_config", "config/fanuc.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    kinematics_yaml = load_yaml(
        "moveit_resources_fanuc_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    test_param = load_yaml("moveit_kinematics", "config/fanuc-lma-test.yaml")

    singular_private_params = {
        "seed": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "num_fk_tests": 0,
        "num_ik_tests": 0,
    }

    singular_unit_tests_poses = {
        "unit_test_poses": {
            "size": 13,
            "pose_0": {
                "pose": [
                    0.889898,
                    -0.201372,
                    1.09478,
                    0.70730173938410212,
                    -0.00017737593464318564,
                    0.7069117395278034,
                    0.00010256596220803817,
                ],
                "joints": [
                    -0.24967849540594997,
                    -0.0064208285266353273,
                    -0.24251650108977454,
                    -2.3129180342434519,
                    -0.34257870605790114,
                    -0.79882125927421033,
                ],
                "type": "absolute",
            },
            "pose_1": {
                "pose": [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                "joints": [
                    0.0,
                    0.17225521580111153,
                    0.18616019238929934,
                    0.0,
                    -0.013904976588187847,
                    0.0,
                ],
                "type": "relative",
            },
            "pose_2": {
                "pose": [-0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                "joints": [
                    0.0,
                    -0.16314862009306849,
                    -0.15067469903041658,
                    0.0,
                    -0.012473921062651848,
                    0.0,
                ],
                "type": "relative",
            },
            "pose_3": {"pose": [0.0, 0.1, 0.0, 0.0, 0.0, 0.0], "type": "relative"},
            "pose_4": {"pose": [0.0, -0.1, 0.0, 0.0, 0.0, 0.0], "type": "relative"},
            "pose_5": {
                "pose": [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
                "joints": [
                    0.0,
                    0.068450807289788598,
                    0.2317804402761828,
                    0.0,
                    -0.16332963298639425,
                    0.0,
                ],
                "type": "relative",
            },
            "pose_6": {
                "pose": [0.0, 0.0, -0.1, 0.0, 0.0, 0.0],
                "joints": [
                    0.0,
                    -0.038270954623974673,
                    -0.19079719308481202,
                    0.0,
                    0.15252623846083735,
                    0.0,
                ],
                "type": "relative",
            },
            "pose_7": {"pose": [0.0, 0.0, 0.0, 0.1, 0.0, 0.0], "type": "relative"},
            "pose_8": {"pose": [0.0, 0.0, 0.0, -0.1, 0.0, 0.0], "type": "relative"},
            "pose_9": {
                "pose": [0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
                "joints": [
                    0,
                    -0.0042227733863642921,
                    -0.019776669463347087,
                    0,
                    0.11555389607698333,
                    0,
                ],
                "type": "relative",
            },
            "pose_10": {
                "pose": [0.0, 0.0, 0.0, 0.0, -0.1, 0.0],
                "joints": [
                    0.0,
                    0.0061816323866541508,
                    0.021837005762580415,
                    0.0,
                    -0.1156553733759268,
                    0.0,
                ],
                "type": "relative",
            },
            "pose_11": {"pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.1], "type": "relative"},
            "pose_12": {"pose": [0.0, 0.0, 0.0, 0.0, 0.0, -0.1], "type": "relative"},
        }
    }

    fanuc_lma_singular = Node(
        package="moveit_kinematics",
        executable="test_kinematics_plugin",
        name="fanuc_lma_singular",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            test_param,
            singular_private_params,
            singular_unit_tests_poses,
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                fanuc_lma_singular,
                KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"fanuc_lma_singular": fanuc_lma_singular},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, fanuc_lma_singular):
        proc_info.assertWaitForShutdown(process=fanuc_lma_singular, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
