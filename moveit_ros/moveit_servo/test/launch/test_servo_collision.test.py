import os
import sys
import unittest

import launch_testing.asserts
sys.path.append(os.path.dirname(__file__))
from servo_launch_test_common import generate_servo_test_description 

def generate_test_description():
    return generate_servo_test_description(gtest_name='test_servo_collision',
                                            start_position_path='../config/collision_start_positions.yaml')


class TestGTestProcessActive(unittest.TestCase):

    def test_gtest_run_complete(self, proc_info, servo_gtest, test_container, fake_joint_driver_node):
        proc_info.assertWaitForShutdown(servo_gtest, timeout=4000.0)

@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):

    def test_gtest_pass(self, proc_info, servo_gtest, test_container, fake_joint_driver_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=servo_gtest)
