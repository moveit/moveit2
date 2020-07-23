import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import unittest
import launch_testing.asserts
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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
    # Get parameters using the demo config file
    servo_yaml = load_yaml('moveit_servo', 'config/panda_simulated_config.yaml')
    servo_params = { 'moveit_servo' : servo_yaml }

    # Get URDF and SRDF
    robot_description_config = load_file('moveit_resources', 'panda_description/urdf/panda.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('moveit_resources', 'panda_moveit_config/config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # Is a perfect controller without dynamics
    fake_joint_driver_node = Node(package='fake_joint_driver',
                                  executable='fake_joint_driver_node',
                                  parameters=[os.path.join(get_package_share_directory("moveit_servo"), "config", "panda_controllers.yaml"),
                                              os.path.join(get_package_share_directory("moveit_servo"), "config", "start_positions.yaml"),
                                              robot_description])

    # Component nodes for tf and Servo
    test_container = ComposableNodeContainer(
            name='test_servo_integration_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_state_publisher',
                    plugin='robot_state_publisher::RobotStatePublisher',
                    name='robot_state_publisher',
                    parameters=[robot_description]),
                ComposableNode(
                    package='tf2_ros',
                    plugin='tf2_ros::StaticTransformBroadcasterNode',
                    name='static_tf2_broadcaster',
                    parameters=[ {'/child_frame_id' : 'panda_link0', '/frame_id' : 'world'} ]),
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoServer',
                    name='servo_server',
                    parameters=[servo_params, robot_description, robot_description_semantic])
            ],
            output='screen',
    )

    integration_gtest = launch_testing.actions.GTest(
        path=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), 'test_servo_integration']),
        timeout=4000.0, output='screen'
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='test_binary_dir',
                                             description='Binary directory of package '
                                                         'containing test executables'),
        fake_joint_driver_node,
        test_container,
        integration_gtest,
        launch_testing.actions.ReadyToTest()
    ]), {'test_container': test_container, 'integration_gtest': integration_gtest, 'fake_joint_driver_node': fake_joint_driver_node}

class TestGTestProcessActive(unittest.TestCase):

    def test_gtest_run_complete(self, proc_info, integration_gtest, test_container, fake_joint_driver_node):
        proc_info.assertWaitForShutdown(integration_gtest, timeout=4000.0)

@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):

    def test_gtest_pass(self, proc_info, integration_gtest, test_container, fake_joint_driver_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=integration_gtest)
