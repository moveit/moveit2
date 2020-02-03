import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = get_package_share_directory('run_moveit_cpp') + "/config/moveit_cpp.yaml"

    # Component yaml files are stored inside dicts for now, allowing grouping the configs in separate namespaces
    robot_description_config = load_file('moveit_resources', 'panda_description/urdf/panda.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('moveit_resources', 'panda_moveit_config/config/panda.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('moveit_resources', 'panda_moveit_config/config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    # This config file had to be migrated, so this is temporarily in the config folder of run_moveit_cpp
    controllers_yaml = load_yaml('run_moveit_cpp', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml }

    ompl_planning_yaml = load_yaml('moveit_resources', 'panda_moveit_config/config/ompl_planning.yaml')
    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        # TODO(henningkayser): enable planning request adapters
        #'request_adapters' : """default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'request_adapters' : '',
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Specify run_moveit_cpp node
    run_moveit_cpp_node = Node(node_name='run_moveit_cpp',
                               package='run_moveit_cpp',
                               # NOTE: not clear how to interact with the gdb instance
                               #prefix='gdb -ex run --args',
                               node_executable='run_moveit_cpp',
                               output='screen',
                               parameters=[moveit_cpp_yaml_file_name,
                                           robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           moveit_controllers])
    return LaunchDescription([ run_moveit_cpp_node ])
