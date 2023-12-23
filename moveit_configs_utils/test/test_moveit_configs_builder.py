from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder.utils import ParameterBuilderFileNotFoundError
import pytest
import unittest


class TestConfigsBuilder(unittest.TestCase):
    def setUp(self):
        self.config_pkgs = ["moveit_resources_fanuc", "moveit_resources_panda"]

    def test_initialization(self):
        for config_pkg in self.config_pkgs:
            moveit_config = MoveItConfigsBuilder(config_pkg)
            self.assertIsInstance(moveit_config, MoveItConfigsBuilder, 
                                  "Initialization Failed.")

    def test_unsuccessful(self):
        # Check that passing an invalid package name fails
        with self.assertRaises(ValueError) as error:
            print(error)
            moveit_config = MoveItConfigsBuilder("Invalid Package")
            self.assertEqual("'Invalid Package' is not a valid package name.", error)
            moveit_config.to_dict()

        # Check that setting an improper field param wrong (e.g. bad file) throws an error.
        moveit_config = MoveItConfigsBuilder(self.config_pkgs[1])
        with self.assertRaises(ParameterBuilderFileNotFoundError):
            moveit_config.robot_description_semantic(file_path="config/doesnt.exist.srdf")
 
    def test_successful(self):
        for config_pkg in self.config_pkgs:
            moveit_config = MoveItConfigsBuilder(self.config_pkgs[0])
            config_dict = moveit_config.to_dict()
            print(config_dict)
            self.assertIsInstance(config_dict, dict, "to_dict() failed to return a dict.")
            required_keys = ["robot_description", 
                             "robot_description_semantic", 
                             "robot_description_kinematics",
                             "robot_description_planning",
                             "planning_pipelines",
                             "publish_planning_scene",
                             "moveit_controller_manager",
                             "trajectory_execution",
                             "ompl",
                             "stomp",
                             "pilz_industrial_motion_planner",
                            ]
            for key in required_keys:
                self.assertIn(key, config_dict, f"No {key} in config for package {config_pkg}.")
    
    def test_panda_with_gripper_config(self):
        moveit_config = (
            MoveItConfigsBuilder("moveit_resources_panda")
            .robot_description(file_path="config/panda.urdf.xacro")
            .robot_description_semantic(file_path="config/panda.srdf")
            .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        )
        config_dict = moveit_config.to_dict()
        self.assertIsInstance(config_dict, dict, "to_dict() failed to return a dict.")

    def test_full_moveit_config(self):
        moveit_configs = (
            # TODO: do this differently than the other tests.
            MoveItConfigsBuilder("moveit_resources_panda")
            #TODO: make this just take in a robot_description_dict.
            .robot_description(
                file_path="config/panda.urdf.xacro",
                # TODO: try mappings?
            )
            .robot_description_semantic(file_path="config/panda.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
            .planning_pipelines(
                "ompl",
                ["pilz_industrial_motion_planner", "chomp", "ompl"],
            )
            .to_moveit_configs()
        )
        print(moveit_configs)


    # TODO: add new test with a xacro in a different package (e.g. a "description" package).
    # TODO: add new test where we pass in the robot description itself.