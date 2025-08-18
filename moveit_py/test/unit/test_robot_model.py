import unittest
from test_moveit.core.robot_model import JointModelGroup, RobotModel

# TODO (peterdavidfagan): depend on moveit_resources package directly.
# (https://github.com/peterdavidfagan/moveit2/blob/moveit_py/moveit_core/utils/src/robot_model_test_utils.cpp)

import os

dir_path = os.path.dirname(os.path.realpath(__file__))
URDF_FILE = "{}/fixtures/panda.urdf".format(dir_path)
SRDF_FILE = "{}/fixtures/panda.srdf".format(dir_path)


def get_robot_model():
    """Helper function that returns a RobotModel instance."""
    return RobotModel(urdf_xml_path=URDF_FILE, srdf_xml_path=SRDF_FILE)


class TestRobotModel(unittest.TestCase):
    def test_initialization(self):
        """
        Test that the RobotModel can be initialized with xml filepaths.
        """
        robot = get_robot_model()
        self.assertIsInstance(robot, RobotModel)

    def test_name_property(self):
        """
        Test that the RobotModel name property returns the correct name.
        """
        robot = get_robot_model()
        self.assertEqual(robot.name, "panda")

    def test_model_frame_property(self):
        """
        Test that the RobotModel model_frame property returns the correct name.
        """
        robot = get_robot_model()
        self.assertEqual(robot.model_frame, "world")

    def test_root_joint_name_property(self):
        """
        Test that the RobotModel root_link property returns the correct name.
        """
        robot = get_robot_model()
        self.assertEqual(robot.root_joint_name, "virtual_joint")

    def test_joint_model_group_names_property(self):
        """
        Test that the RobotModel joint_model_group_names property returns the correct names.
        """
        robot = get_robot_model()
        self.assertCountEqual(
            robot.joint_model_group_names, ["panda_arm", "hand", "panda_arm_hand"]
        )

    def test_joint_model_groups_property(self):
        """
        Test that the RobotModel joint_model_groups returns a list of JointModelGroups.
        """
        robot = get_robot_model()
        self.assertIsInstance(robot.joint_model_groups[0], JointModelGroup)

    def test_has_joint_model_group(self):
        """
        Test that the RobotModel has_joint_model_group returns True for existing groups.
        """
        robot = get_robot_model()
        self.assertTrue(robot.has_joint_model_group("panda_arm"))
        self.assertFalse(robot.has_joint_model_group("The answer is 42."))

    def test_get_joint_model_group(self):
        """
        Test that the RobotModel get_joint_model_group returns the correct group.
        """
        robot = get_robot_model()
        jmg = robot.get_joint_model_group("panda_arm")
        self.assertIsInstance(jmg, JointModelGroup)
        self.assertEqual(jmg.name, "panda_arm")


if __name__ == "__main__":
    unittest.main()
