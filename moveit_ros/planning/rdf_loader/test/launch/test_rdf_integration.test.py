from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing

import unittest


def generate_test_description():
    rdf_path = get_package_share_path("moveit_ros_planning") / "rdf_loader"
    test_data_path = rdf_path / "test" / "data"

    kermit_urdf = open(test_data_path / "kermit.urdf").read()
    kermit_srdf = open(test_data_path / "kermit.srdf").read()

    boring_urdf_node = Node(
        package="moveit_ros_planning",
        executable="boring_string_publisher",
        name="urdf_pub0",
        parameters=[
            {
                "topic": "topic_description",
                "content_path": str(test_data_path / "gonzo.urdf"),
            }
        ],
    )
    boring_srdf_node = Node(
        package="moveit_ros_planning",
        executable="boring_string_publisher",
        name="srdf_pub0",
        parameters=[
            {
                "topic": "topic_description_semantic",
                "content_path": str(test_data_path / "gonzo.srdf"),
            }
        ],
    )

    integration_node = Node(
        package="moveit_ros_planning",
        executable="test_rdf_integration",
        parameters=[
            {
                "robot_description": kermit_urdf,
                "robot_description_semantic": kermit_srdf,
                # The following timeouts are not strictly needed, but make the test run quicker
                "DNE_timeout": 1.0,
                "DNE_semantic_timeout": 1.0,
            }
        ],
    )

    return (
        LaunchDescription(
            [
                integration_node,
                boring_urdf_node,
                boring_srdf_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"integration_node": integration_node},
    )


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, integration_node):
        proc_info.assertWaitForShutdown(integration_node, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, integration_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=integration_node)
