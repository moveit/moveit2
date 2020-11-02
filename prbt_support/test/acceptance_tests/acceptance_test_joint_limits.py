#!/usr/bin/env python
# Copyright (c) 2018 Pilz GmbH & Co. KG
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

from __future__ import print_function

import unittest
from math import radians

import rospy
from urdf_parser_py.urdf import URDF
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

_CONTROLLER_ACTION_NAME = '/prbt/manipulator_joint_trajectory_controller/follow_joint_trajectory'
_ACTION_SERVER_TIMEOUT_SEC = 5
_JOINT_STATES_TOPIC_NAME = '/joint_states'
_WAIT_FOR_MESSAGE_TIMEOUT_SEC = 1

# Use axis ranges from data sheet: https://www.pilz.com/download/open/PRBT_6_Operat_Manual_1004685-EN-02.pdf (page 19)
_JOINT_LIMITS_DEGREE = {
    'prbt_joint_1': 170,
    'prbt_joint_2': 145,
    'prbt_joint_3': 135,
    'prbt_joint_4': 170,
    'prbt_joint_5': 170,
    'prbt_joint_6': 179
}
_JOINT_POSITIONS_TOLERANCE = 0.001
_VELOCITY_SCALE = 0.2
_JOINT_LIMIT_OVERSTEP = 0.1
_SELF_COLLISION_JOINT_NAME = 'prbt_joint_5'


class AcceptanceTestJointLimits(unittest.TestCase):
    """ Test that each joint can be moved exactly to its limits and not further.
    """

    def setUp(self):
        """ Initialize a client for sending trajectories to the controller.
        """
        self.client = SimpleActionClient(_CONTROLLER_ACTION_NAME, FollowJointTrajectoryAction)
        if not self.client.wait_for_server(timeout=rospy.Duration(_ACTION_SERVER_TIMEOUT_SEC)):
            self.fail('Timed out waiting for action server ' + _CONTROLLER_ACTION_NAME)

        self.joint_names = sorted(_JOINT_LIMITS_DEGREE.keys())
        self.home_positions = [0] * len(self.joint_names)

        # read joint limits from urdf
        self.joint_velocity_limits = [0] * len(self.joint_names)
        robot = URDF.from_parameter_server("robot_description")

        for joint in robot.joints:
            if joint.name in self.joint_names:
                index = self.joint_names.index(joint.name)
                self.joint_velocity_limits[index] = joint.limit.velocity

        self.assertTrue(all(self.joint_velocity_limits))

    def _ask_for_permission(self, test_name):
        s = raw_input('Perform ' + test_name + ' [(y)es, (n)o]?: ')
        if(s == "n"):
            print('\n\nSkip ' + test_name + '\n___TEST-END___\n')
            return 0
        print('\n\nStart ' + test_name + '\n')
        return 1

    def _current_joint_state_positions(self):
        """ Return the current joint state positions in the order given by self.joint_names.
        """
        msg = JointState()
        positions = []
        try:
            msg = rospy.wait_for_message(_JOINT_STATES_TOPIC_NAME, JointState, timeout=_WAIT_FOR_MESSAGE_TIMEOUT_SEC)
        except rospy.ROSException:
            self.fail('Could not retrieve message from topic ' + _JOINT_STATES_TOPIC_NAME)

        for name in self.joint_names:
            try:
                index = msg.name.index(name)
                positions.append(msg.position[index])
            except ValueError:
                self.fail('Could not retrieve joint state position for ' + name)

        return positions

    def _check_joint_limits(self):
        """ Check if current joint positions are within the limits.
        """
        positions = self._current_joint_state_positions()

        for i in range(len(self.joint_names)):
            position = positions[i]
            name = self.joint_names[i]
            limit = radians(_JOINT_LIMITS_DEGREE[name])+_JOINT_POSITIONS_TOLERANCE

            self.assertGreater(position, -limit, 'Joint ' + name + ' violates lower limit. Position: ' + str(position))
            self.assertLess(position, limit, 'Joint ' + name + ' violates upper limit. Position: ' + str(position))

    def _drive_home(self):
        """ Move the robot to the home position.
        """
        positions = self._current_joint_state_positions()
        diffs = []
        for i in range(len(positions)):
            diffs.append(abs(positions[i]-self.home_positions[i]))

        if any(diff > _JOINT_POSITIONS_TOLERANCE for diff in diffs):
            self._execute_trajectory(self.home_positions)

    def _execute_trajectory(self, positions):
        """ Execute a single point trajectory given through joint positions (in the order given by self.joint_names).
            Return true on success and false otherwise.
        """
        self.assertEqual(len(positions), len(self.joint_names))

        # determine duration from distance, max velocity and velocity scaling
        current_positions = self._current_joint_state_positions()
        durations = []

        for i in range(len(current_positions)):
            distance = abs(positions[i] - current_positions[i])
            durations.append(distance/self.joint_velocity_limits[i])

        # we cannot assume constant velocity, so multiply by a factor of 2 to compensate for acceleration/deceleration
        duration = 2*max(durations)/_VELOCITY_SCALE

        # construct goal
        traj_point = JointTrajectoryPoint()
        traj_point.positions = positions
        traj_point.velocities = [0.0]*len(positions)
        traj_point.time_from_start = rospy.Duration(duration)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(traj_point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj

        # send to trajectory controller
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # evaluate result
        result = self.client.get_result()
        success = (result.error_code == FollowJointTrajectoryResult.SUCCESSFUL)
        if not success:
            rospy.loginfo('Failure executing: ' + str(goal))
        return success

    def _joint_limit_reaching_test(self, joint_name):
        """ Test if the robot can be commanded to move exactly to the limits

            Test Sequence:
              1. Command a movement to the home position.
              2. Command a movement to the lower limit.
              3. Command a movement to the upper limit.
              4. Command a movement to the home position.

            Expected Results:
              1. Trajectory is executed successfully.
              2. Trajectory is executed successfully.
              3. Trajectory is executed successfully.
              4. Trajectory is executed successfully.
        """
        self._drive_home()

        index = self.joint_names.index(joint_name)
        limit = _JOINT_LIMITS_DEGREE[joint_name]

        lower_positions = [0] * len(self.joint_names)
        lower_positions[index] = -radians(limit)

        self.assertTrue(self._execute_trajectory(lower_positions))

        upper_positions = [0] * len(self.joint_names)
        upper_positions[index] = radians(limit)

        self.assertTrue(self._execute_trajectory(upper_positions))

        self._drive_home()

    def _joint_limit_overstepping_test(self, joint_name):
        """ Test if the robot does not overstep the limits

            Test Sequence:
              1. Command a movement to the home position.
              2. Command a movement overstepping the lower limit.
              3. Command a movement overstepping the upper limit.
              4. Command a movement to the home position.

            Expected Results:
              1. Trajectory is executed successfully.
              2. Trajectory execution is aborted and the robot does not overstep the limits.
              3. Trajectory execution is aborted and the robot does not overstep the limits.
              4. Trajectory is executed successfully.
        """
        self._drive_home()

        index = self.joint_names.index(joint_name)
        limit = _JOINT_LIMITS_DEGREE[joint_name]

        lower_positions = [0] * len(self.joint_names)
        lower_positions[index] = -(radians(limit) + _JOINT_LIMIT_OVERSTEP)

        self.assertFalse(self._execute_trajectory(lower_positions))
        self._check_joint_limits()

        upper_positions = [0] * len(self.joint_names)
        upper_positions[index] = radians(limit) + _JOINT_LIMIT_OVERSTEP

        self.assertFalse(self._execute_trajectory(upper_positions))
        self._check_joint_limits()

        self._drive_home()

    def test_joint_limits_reaching(self):
        """ Perform all reaching tests. Before each test ask the user if he wants to skip it.
        """
        for name in self.joint_names:
            if not name == _SELF_COLLISION_JOINT_NAME:
                if self._ask_for_permission('joint_limit_reaching_test for ' + name):
                    self._joint_limit_reaching_test(name)

    def test_joint_limits_overstepping(self):
        """ Perform all overstepping tests. Before each test ask the user if he wants to skip it.
        """
        for name in self.joint_names:
            if not name == _SELF_COLLISION_JOINT_NAME:
                if self._ask_for_permission('joint_limit_overstepping_test for ' + name):
                    self._joint_limit_overstepping_test(name)


if __name__ == "__main__":
    # init ros node
    rospy.init_node('acceptance_test_joint_limits_node')
    unittest.main()
