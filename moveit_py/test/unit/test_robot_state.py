import unittest
import numpy as np

from geometry_msgs.msg import Pose

from test_moveit.core.robot_state import RobotState
from test_moveit.core.robot_model import RobotModel


# TODO (peterdavidfagan): depend on moveit_resources package directly.
# (https://github.com/peterdavidfagan/moveit2/blob/moveit_py/moveit_core/utils/src/robot_model_test_utils.cpp)

import os

dir_path = os.path.dirname(os.path.realpath(__file__))
URDF_FILE = "{}/fixtures/panda.urdf".format(dir_path)
SRDF_FILE = "{}/fixtures/panda.srdf".format(dir_path)


def get_robot_model():
    """Helper function that returns a RobotModel instance."""
    return RobotModel(urdf_xml_path=URDF_FILE, srdf_xml_path=SRDF_FILE)


class TestRobotState(unittest.TestCase):
    def test_initialization(self):
        """
        Test that RobotState can be initialized with a RobotModel
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)

        self.assertIsInstance(robot_state, RobotState)

    def test_robot_model_property(self):
        """
        Test that the robot_model property returns the correct RobotModel
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)

        self.assertEqual(robot_state.robot_model, robot_model)

    def test_get_frame_transform(self):
        """
        Test that the frame transform is correct
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        frame_transform = robot_state.get_frame_transform("panda_link0")

        self.assertIsInstance(frame_transform, np.ndarray)
        # TODO(peterdavidfagan): add assertion for particular values

    def test_get_pose(self):
        """
        Test that the pose is correct
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        pose = robot_state.get_pose(link_name="panda_link8")

        self.assertIsInstance(pose, Pose)
        # TODO(peterdavidfagan): add assertion for particular values

    def test_get_jacobian_1(self):
        """
        Test that the jacobian is correct
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        jacobian = robot_state.get_jacobian(
            joint_model_group_name="panda_arm",
            reference_point_position=np.array([0.0, 0.0, 0.0]),
        )

        self.assertIsInstance(jacobian, np.ndarray)
        # TODO(peterdavidfagan): add assertion for particular values

    def test_get_jacobian_2(self):
        """
        Test that the jacobian is correct
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        jacobian = robot_state.get_jacobian(
            joint_model_group_name="panda_arm",
            link_name="panda_link6",
            reference_point_position=np.array([0.0, 0.0, 0.0]),
        )

        self.assertIsInstance(jacobian, np.ndarray)
        # TODO(peterdavidfagan): add assertion for particular values

    def test_set_joint_group_positions(self):
        """
        Test that the joint group positions can be set
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        joint_group_positions = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        robot_state.set_joint_group_positions(
            joint_model_group_name="panda_arm", position_values=joint_group_positions
        )

        self.assertEqual(
            joint_group_positions.tolist(),
            robot_state.get_joint_group_positions("panda_arm").tolist(),
        )

    def test_set_joint_group_positions_from_list(self):
        """Python sequences are accepted via NumPy forcecast conversion."""
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        joint_group_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        robot_state.set_joint_group_positions(
            joint_model_group_name="panda_arm", position_values=joint_group_positions
        )

        np.testing.assert_allclose(
            joint_group_positions,
            robot_state.get_joint_group_positions("panda_arm"),
        )

    def test_set_joint_group_active_positions(self):
        """Active-joint setter must accept NumPy input (issue #3718 crash path)."""
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        joint_group_positions = np.array(
            [0.0, 0.0, 1.57, 0.0, 1.57, 0.0, 0.0], dtype=np.float32
        )
        robot_state.set_joint_group_active_positions(
            "panda_arm",
            joint_group_positions,
        )

        np.testing.assert_allclose(
            joint_group_positions,
            robot_state.get_joint_group_positions("panda_arm"),
        )

    def test_set_joint_group_positions_from_column_array(self):
        """Preserve Eigen VectorXd's support for n-by-1 NumPy arrays."""
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        joint_group_positions = np.arange(7, dtype=np.int32).reshape(-1, 1)

        robot_state.set_joint_group_positions(
            "panda_arm",
            joint_group_positions,
        )

        np.testing.assert_allclose(
            joint_group_positions.ravel(),
            robot_state.get_joint_group_positions("panda_arm"),
        )

    def test_set_joint_group_positions_reject_invalid_shapes_and_sizes(self):
        """Invalid input must raise instead of reaching unchecked C++ pointer reads."""
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()

        with self.assertRaises(ValueError):
            robot_state.set_joint_group_positions("panda_arm", [0.0] * 6)
        with self.assertRaises(ValueError):
            robot_state.set_joint_group_active_positions("panda_arm", [0.0] * 6)
        with self.assertRaises(ValueError):
            robot_state.set_joint_group_positions(
                "panda_arm",
                np.zeros((1, 7)),
            )

    def test_set_joint_group_velocities(self):
        """
        Test that the joint group velocities can be set
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        joint_group_velocities = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        robot_state.set_joint_group_velocities(
            joint_model_group_name="panda_arm", velocity_values=joint_group_velocities
        )

        self.assertEqual(
            joint_group_velocities.tolist(),
            robot_state.get_joint_group_velocities("panda_arm").tolist(),
        )

    def test_set_joint_group_accelerations(self):
        """
        Test that the joint group accelerations can be set
        """
        robot_model = get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        joint_group_accelerations = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        robot_state.set_joint_group_accelerations(
            joint_model_group_name="panda_arm",
            acceleration_values=joint_group_accelerations,
        )

        self.assertEqual(
            joint_group_accelerations.tolist(),
            robot_state.get_joint_group_accelerations("panda_arm").tolist(),
        )

    # TODO (peterdavidfagan): requires kinematics solver to be loaded
    # def test_set_from_ik(self):
    #    """
    #    Test that the robot state can be set from an IK solution
    #    """
    #    robot_model = RobotModel(
    #        urdf_xml_path="./fixtures/panda.urdf", srdf_xml_path="./fixtures/panda.srdf"
    #    )
    #    robot_state = RobotState(robot_model)
    #    robot_state.update()
    #    pose = Pose()
    #    pose.position.x = 0.5
    #    pose.position.y = 0.5
    #    pose.position.z = 0.5
    #    pose.orientation.w = 1.0

    #    robot_state.set_from_ik(
    #        joint_model_group_name="panda_arm",
    #        geometry_pose=pose,
    #        tip_name="panda_link8",
    #    )

    #    self.assertEqual(robot_state.get_pose("panda_link8"), pose)


if __name__ == "__main__":
    unittest.main()
