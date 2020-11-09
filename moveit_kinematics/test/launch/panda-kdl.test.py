import os
import yaml
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from ament_index_python.packages import get_package_share_directory

import pytest

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.full_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

@pytest.mark.rostest
def generate_test_description():
    #TODO Add panda_arm_singular test ()

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('moveit_resources_panda_description', 'urdf/panda.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('moveit_resources_panda_moveit_config', 'config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}
    kinematics_yaml = load_yaml('moveit_resources_panda_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }
    test_param = load_yaml('moveit_kinematics', 'config/panda-kdl-test.yaml')
    test_node = Node(package='moveit_kinematics',
                     executable='test_kinematics_plugin',
                     name='test_kinmatics_plugin',
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 robot_description_kinematics,
                                 test_param
                                 ]
    )
    return LaunchDescription([
        test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node}

# Workaround for https://github.com/ros2/launch/issues/380
#TODO Remove workaround once its fixed
class TestLoggingOutputFormat(unittest.TestCase):

    def test_logging_output(self, proc_info, proc_output, test_node):
        proc_info.assertWaitForShutdown(process=test_node, timeout=10.0)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

