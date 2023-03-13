import unittest

from moveit_py.core import JointModelGroup, RobotModel

# TODO (peterdavidfagan): depend on moveit_resources package directly.
# (https://github.com/peterdavidfagan/moveit2/blob/moveit_py/moveit_core/utils/src/robot_model_test_utils.cpp)
class TestRobotModel(unittest.TestCase):
    def test_initialization(self):
        """
        Test that the RobotModel can be initialized with xml filepaths.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertIsInstance(robot, RobotModel)

    def test_name_property(self):
        """
        Test that the RobotModel name property returns the correct name.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertEqual(robot.name, "panda")

    def test_model_frame_property(self):
        """
        Test that the RobotModel model_frame property returns the correct name.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertEqual(robot.model_frame, "world")

    def test_root_joint_name_property(self):
        """
        Test that the RobotModel root_link property returns the correct name.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertEqual(robot.root_joint_name, "virtual_joint")

    def test_joint_model_group_names_property(self):
        """
        Test that the RobotModel joint_model_group_names property returns the correct names.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertCountEqual(
            robot.joint_model_group_names, ["panda_arm", "hand", "panda_arm_hand"]
        )

    def test_joint_model_groups_property(self):
        """
        Test that the RobotModel joint_model_groups returns a list of JointModelGroups.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertIsInstance(robot.joint_model_groups[0], JointModelGroup)

    def test_has_joint_model_group(self):
        """
        Test that the RobotModel has_joint_model_group returns True for existing groups.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        self.assertTrue(robot.has_joint_model_group("panda_arm"))
        self.assertFalse(robot.has_joint_model_group("The answer is 42."))

    def test_get_joint_model_group(self):
        """
        Test that the RobotModel get_joint_model_group returns the correct group.
        """
        robot = RobotModel(
            urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
        )
        jmg = robot.get_joint_model_group("panda_arm")
        self.assertIsInstance(jmg, JointModelGroup)
        self.assertEqual(jmg.name, "panda_arm")


if __name__ == "__main__":
    unittest.main()
