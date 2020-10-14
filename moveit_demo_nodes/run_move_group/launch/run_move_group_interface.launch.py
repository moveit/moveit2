from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_utils.launch_test_utils import load_file, load_yaml


def generate_launch_description():
    # planning_context
    robot_description_config = load_file('moveit_resources_panda_description', 'urdf/panda.urdf')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('moveit_resources_panda_moveit_config', 'config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('moveit_resources_panda_moveit_config', 'config/kinematics.yaml')

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(name='run_move_group',
                               package='run_move_group',
                               executable='run_move_group',
                               prefix='xterm -e',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml])

    return LaunchDescription([run_move_group_demo])
