import os
import launch
import unittest
import launch_ros
import launch_testing
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_ros_tests.move_group_test_description import (
    generate_move_group_test_description,
)


def generate_test_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["default_planning_scene"] = os.path.join(
        get_package_share_directory("moveit_ros_tests"), "data", "scene.txt"
    )

    return generate_move_group_test_description(moveit_config_dict)


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, move_group_gtest):
        self.proc_info.assertWaitForShutdown(move_group_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, move_group_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=move_group_gtest)
