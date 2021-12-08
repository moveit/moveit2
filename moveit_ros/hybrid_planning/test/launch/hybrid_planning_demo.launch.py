import launch
import os
import sys

sys.path.append(os.path.dirname(__file__))
from hybrid_planning_common import generate_common_hybrid_launch_description


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    return launch.LaunchDescription(generate_common_hybrid_launch_description())
