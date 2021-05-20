import launch_testing
import os
import sys
import unittest

sys.path.append(os.path.dirname(__file__))
from servo_launch_test_common import generate_servo_test_description


def generate_test_description():
    return generate_servo_test_description(
        gtest_name="test_servo_singularity",
        start_position_path="../config/singularity_start_positions.yaml",
    )


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, servo_gtest):
        self.proc_info.assertWaitForShutdown(servo_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, servo_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=servo_gtest)
