#!/usr/bin/env python3

# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import os
import sys
import unittest

import launch_testing

sys.path.append(os.path.dirname(__file__))
from generate_minimal_move_group_launch_description import (  # noqa: E402 - futzing with the PATH has to happen first
    generate_minimal_move_group_launch_description,
)


def generate_test_description():
    return generate_minimal_move_group_launch_description()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
