import os
import unittest

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from trajectory_msgs.msg import JointTrajectoryPoint

from test_moveit.core.robot_model import RobotModel
from test_moveit.core.robot_state import RobotState
from test_moveit.core.robot_trajectory import RobotTrajectory

dir_path = os.path.dirname(os.path.realpath(__file__))
URDF_FILE = "{}/fixtures/panda.urdf".format(dir_path)
SRDF_FILE = "{}/fixtures/panda.srdf".format(dir_path)

GROUP = "panda_arm"
ARM_JOINTS = ["panda_joint{}".format(i) for i in range(1, 8)]

# Three arm configurations inside the joint limits of the Panda fixture.
WAYPOINT_POSITIONS = [
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    [0.1, -0.700, 0.1, -2.200, 0.1, 1.700, 0.900],
    [0.2, -0.600, 0.2, -2.000, 0.2, 1.800, 1.000],
]
# time_from_start of each waypoint, in seconds.
WAYPOINT_TIMES = [0.0, 1.0, 3.0]


def get_robot_model():
    """Helper function that returns a RobotModel instance."""
    return RobotModel(urdf_xml_path=URDF_FILE, srdf_xml_path=SRDF_FILE)


def make_trajectory_msg(positions=WAYPOINT_POSITIONS, times=WAYPOINT_TIMES):
    """Build a moveit_msgs RobotTrajectory message for the Panda arm joints."""
    msg = RobotTrajectoryMsg()
    msg.joint_trajectory.joint_names = list(ARM_JOINTS)
    for point_positions, t in zip(positions, times):
        point = JointTrajectoryPoint()
        point.positions = list(point_positions)
        seconds = int(t)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int(round((t - seconds) * 1e9))
        )
        msg.joint_trajectory.points.append(point)
    return msg


def make_trajectory(robot_model, msg=None):
    """Return a RobotTrajectory for the Panda arm, filled from msg."""
    trajectory = RobotTrajectory(robot_model)
    trajectory.joint_model_group_name = GROUP
    reference_state = RobotState(robot_model)
    reference_state.set_to_default_values()
    trajectory.set_robot_trajectory_msg(reference_state, msg or make_trajectory_msg())
    return trajectory


def arm_positions(robot_state):
    """Return the Panda arm joint positions of a RobotState as a list."""
    return list(robot_state.get_joint_group_positions(GROUP))


def to_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class TestRobotTrajectory(unittest.TestCase):
    def test_initialization(self):
        """
        Test that an empty RobotTrajectory can be created from a robot model.
        """
        robot_model = get_robot_model()
        trajectory = RobotTrajectory(robot_model)

        self.assertIsInstance(trajectory, RobotTrajectory)
        self.assertEqual(len(trajectory), 0)
        self.assertEqual(trajectory.duration, 0.0)
        self.assertEqual(trajectory.average_segment_duration, 0.0)
        self.assertEqual(trajectory.get_waypoint_durations(), [])

    def test_robot_model_property(self):
        """
        Test that the robot_model property returns the model the trajectory was created with.
        """
        robot_model = get_robot_model()
        trajectory = RobotTrajectory(robot_model)

        self.assertEqual(trajectory.robot_model.name, "panda")

    def test_joint_model_group_name_property(self):
        """
        Test that the joint_model_group_name property can be set and read back.
        """
        trajectory = RobotTrajectory(get_robot_model())
        trajectory.joint_model_group_name = GROUP

        self.assertEqual(trajectory.joint_model_group_name, GROUP)

    def test_set_robot_trajectory_msg(self):
        """
        Test that setting a trajectory message creates one waypoint per point, with matching positions.
        """
        trajectory = make_trajectory(get_robot_model())

        self.assertEqual(len(trajectory), len(WAYPOINT_POSITIONS))
        for idx, expected in enumerate(WAYPOINT_POSITIONS):
            waypoint = trajectory[idx]
            self.assertIsInstance(waypoint, RobotState)
            for actual, want in zip(arm_positions(waypoint), expected):
                self.assertAlmostEqual(actual, want)

    def test_durations(self):
        """
        Test the waypoint durations, total duration and average segment duration.
        """
        trajectory = make_trajectory(get_robot_model())

        durations = trajectory.get_waypoint_durations()
        self.assertEqual(len(durations), 3)
        for actual, want in zip(durations, [0.0, 1.0, 2.0]):
            self.assertAlmostEqual(actual, want)
        self.assertAlmostEqual(trajectory.duration, 3.0)
        # The first waypoint has a zero duration, so it is left out of the average: 3.0 s / 2 segments.
        self.assertAlmostEqual(trajectory.average_segment_duration, 1.5)

    def test_iteration(self):
        """
        Test that iterating over a trajectory yields (RobotState, duration from previous) pairs in order.
        """
        trajectory = make_trajectory(get_robot_model())

        items = list(trajectory)
        self.assertEqual(len(items), len(WAYPOINT_POSITIONS))
        for (waypoint, duration), expected, want_duration in zip(
            items, WAYPOINT_POSITIONS, [0.0, 1.0, 2.0]
        ):
            self.assertIsInstance(waypoint, RobotState)
            self.assertAlmostEqual(duration, want_duration)
            for actual, want in zip(arm_positions(waypoint), expected):
                self.assertAlmostEqual(actual, want)

    def test_get_robot_trajectory_msg(self):
        """
        Test that a trajectory converts back to a message with the same joints, positions and timing.
        """
        trajectory = make_trajectory(get_robot_model())
        msg = trajectory.get_robot_trajectory_msg()

        self.assertEqual(list(msg.joint_trajectory.joint_names), ARM_JOINTS)
        self.assertEqual(len(msg.joint_trajectory.points), len(WAYPOINT_POSITIONS))
        for point, expected, t in zip(
            msg.joint_trajectory.points, WAYPOINT_POSITIONS, WAYPOINT_TIMES
        ):
            for actual, want in zip(point.positions, expected):
                self.assertAlmostEqual(actual, want)
            self.assertAlmostEqual(to_seconds(point.time_from_start), t)

    def test_get_robot_trajectory_msg_joint_filter(self):
        """
        Test that joint_filter limits the message to the requested joints.
        """
        trajectory = make_trajectory(get_robot_model())
        msg = trajectory.get_robot_trajectory_msg(
            joint_filter=["panda_joint1", "panda_joint7"]
        )

        self.assertEqual(
            list(msg.joint_trajectory.joint_names), ["panda_joint1", "panda_joint7"]
        )
        for point, expected in zip(msg.joint_trajectory.points, WAYPOINT_POSITIONS):
            self.assertEqual(len(point.positions), 2)
            self.assertAlmostEqual(point.positions[0], expected[0])
            self.assertAlmostEqual(point.positions[1], expected[6])

    def test_reverse(self):
        """
        Test that __reverse__ reverses the order of the waypoints in place.
        """
        trajectory = make_trajectory(get_robot_model())
        trajectory.__reverse__()

        self.assertEqual(len(trajectory), len(WAYPOINT_POSITIONS))
        for idx, expected in enumerate(reversed(WAYPOINT_POSITIONS)):
            for actual, want in zip(arm_positions(trajectory[idx]), expected):
                self.assertAlmostEqual(actual, want)

    def test_unwind_without_continuous_joints(self):
        """
        Test that unwind leaves positions unchanged when the group has no continuous joints.
        """
        trajectory = make_trajectory(get_robot_model())
        trajectory.unwind()

        for idx, expected in enumerate(WAYPOINT_POSITIONS):
            for actual, want in zip(arm_positions(trajectory[idx]), expected):
                self.assertAlmostEqual(actual, want)

    def test_apply_totg_fails_without_acceleration_limits(self):
        """
        Test that TOTG reports failure, and leaves the trajectory untimed, when the model has no acceleration limits.

        The Panda fixture URDF defines velocity limits only, and acceleration limits cannot come from URDF.
        """
        untimed = make_trajectory_msg(times=[0.0, 0.0, 0.0])
        trajectory = make_trajectory(get_robot_model(), untimed)

        self.assertFalse(trajectory.apply_totg_time_parameterization(1.0, 1.0))

        self.assertEqual(len(trajectory), len(WAYPOINT_POSITIONS))
        self.assertEqual(trajectory.duration, 0.0)

    def test_apply_ruckig_smoothing(self):
        """
        Test that Ruckig smoothing succeeds on a timed trajectory and keeps its start and end configurations.

        The Panda fixture has no acceleration or jerk limits, so Ruckig falls back to its default limits.
        """
        trajectory = make_trajectory(get_robot_model())

        self.assertTrue(trajectory.apply_ruckig_smoothing(1.0, 1.0))

        self.assertGreaterEqual(len(trajectory), 2)
        self.assertGreater(trajectory.duration, 0.0)
        durations = trajectory.get_waypoint_durations()
        self.assertTrue(all(d > 0.0 for d in durations[1:]))
        for actual, want in zip(arm_positions(trajectory[0]), WAYPOINT_POSITIONS[0]):
            self.assertAlmostEqual(actual, want, places=4)
        for actual, want in zip(
            arm_positions(trajectory[len(trajectory) - 1]), WAYPOINT_POSITIONS[-1]
        ):
            self.assertAlmostEqual(actual, want, places=4)


if __name__ == "__main__":
    unittest.main()
