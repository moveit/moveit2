import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import unittest
import launch_testing.asserts
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch

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
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_test_description():
    # Get (valid) parameters using the demo config file
    servo_yaml = load_yaml('moveit_servo', 'config/panda_simulated_config.yaml')
    servo_params = { 'moveit_servo' : servo_yaml }

    param_node_valid = Node(
        package='moveit_servo',
        executable=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), 'test_servo_parameters_node']),
        output='screen',
        name='valid_parameter_load',
        parameters=[ servo_params, {'expect_valid_params': True} ]
    )

    param_gtest = launch_testing.actions.GTest(
        path=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), 'test_servo_parameters']),
        timeout=4000.0, output='screen'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='test_binary_dir',
                                             description='Binary directory of package '
                                                         'containing test executables'),
        param_node_valid,
        param_gtest,
        launch_testing.actions.ReadyToTest()
    ]), {'param_node_valid': param_node_valid, 'param_gtest': param_gtest}


class TestGTestProcessActive(unittest.TestCase):

    def test_gtest_run_complete(self, proc_info, param_gtest, param_node_valid):
        proc_info.assertWaitForShutdown(param_gtest, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):

    def test_gtest_pass(self, proc_info, param_gtest, param_node_valid):
        launch_testing.asserts.assertExitCodes(proc_info, process=param_gtest)
