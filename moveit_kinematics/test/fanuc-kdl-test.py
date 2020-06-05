import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
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
            return yaml.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

@pytest.mark.rostest
def generate_test_description():

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('moveit_resources', 'fanuc_description/urdf/fanuc.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('moveit_resources', 'fanuc_moveit_config/config/fanuc.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('moveit_resources', 'fanuc_moveit_config/config/kinematics.yaml')
    kinematics_yaml['manipulator']['max_solver_iterations'] = 100 # does this work?
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    tolerance = 0.1  ## TODO(henningkayser): set arg
    test_params = {
        'tip_link': 'tool0',
        'root_link': 'base_link',
        'group': 'manipulator',
        'ik_timeout': 0.2,
        'tolerance': tolerance,
        'joint_names': [ 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6' ],
    }

    ik_plugin = 'kdl_kinematics_plugin/KDLKinematicsPlugin'  # TODO(henningkayser): set arg
    kdl_params = {
        'ik_plugin_name': ik_plugin,
        'num_fk_tests': 100,
        'num_ik_tests': 100,
        'num_ik_cb_tests': 0,
        'num_ik_multipletests': 0,
        'num_nearest_ik_tests': 0,
    }

    private_params = {
        'seed': [0, -0.32, -0.5, 0, -0.5, 0],
        'consistency_limits': [0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
    }

    unit_tests_poses = {
        'unit_test_poses':
        {
            'size': 6,
            'pose_1': { 'pose':   [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'joints': [0, -0.152627, -0.367847, 0, -0.46478, 0],
                        'type':  'relative'
                      },
            'pose_2': { 'pose':   [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
                        'joints': [0.1582256, -0.3066389, -0.490349, 0.250946, -0.5159858, -0.319381],
                        'type':   'relative'
                      },
            'pose_3': { 'pose':   [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
                        'joints': [0, -0.287588, -0.324304, 0, -0.643285, 0],
                        'type':   'relative'
                      },
            'pose_4': { 'pose':   [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'joints': [-0.0159181, -0.319276, -0.499953, -0.231014, -0.511806, 0.212341],
                        'type':   'relative'
                      },
            'pose_5': { 'pose':   [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'joints': [0, -0.331586, -0.520375, 0, -0.391211, 0],
                        'type':   'relative'
                      },
            'pose_6': { 'pose':   [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'joints': [0, -0.32, -0.5, 0, -0.5, -0.1],
                        'type':   'relative'
                      },
        }
    }

    test_node = Node(package='moveit_kinematics',
                     node_executable='test_kinematics_plugin',
                     node_name='test_kinmatics_plugin',
                     # prefix='xterm -e gdb --args',
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 robot_description_kinematics,
                                 test_params,
                                 kdl_params,
                                 private_params])

    return LaunchDescription([ test_node, launch_testing.actions.ReadyToTest() ])

