import os
import sys
import unittest

import launch_testing.asserts

sys.path.append(os.path.dirname(__file__))
from move_group_launch_test_common import generate_move_group_test_description


def generate_test_description():
    return generate_move_group_test_description(
        gtest_name="move_group_ompl_constraints_test"
    )


class TestGTestProcessActive(unittest.TestCase):
    def test_gtest_run_complete(
        self,
        proc_info,
        ompl_constraint_test,
        run_move_group_node,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
    ):
        proc_info.assertWaitForShutdown(ompl_constraint_test, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(
        self,
        proc_info,
        ompl_constraint_test,
        run_move_group_node,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
    ):
        launch_testing.asserts.assertExitCodes(proc_info, process=ompl_constraint_test)
