import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_helpers.get_parameters_module import get_parameters_module, load_file, load_yaml

def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = get_package_share_directory('run_moveit_cpp') + "/config/moveit_cpp.yaml"

    parameters = {}
    parameters.update(get_parameters_module('moveit_resources', '/panda_moveit_config/launch/planning_context.launch.py')(load_robot_description=True))
    parameters.update(get_parameters_module('moveit_resources', '/panda_moveit_config/launch/planning_pipeline.launch.py')())
    parameters.update(get_parameters_module('moveit_resources', '/panda_moveit_config/launch/panda_moveit_controller_manager.launch.py')())

    # MoveItCpp demo executable
    run_moveit_cpp_node = Node(node_name='run_moveit_cpp',
                               package='run_moveit_cpp',
                               # TODO(henningkayser): add debug argument
                               # prefix='xterm -e gdb --args',
                               node_executable='run_moveit_cpp',
                               output='screen',
                               parameters=[moveit_cpp_yaml_file_name,
                                           parameters])

    # RViz
    rviz_config_file = get_package_share_directory('run_moveit_cpp') + "/launch/run_moveit_cpp.rviz"
    rviz_node = Node(package='rviz2',
                     node_executable='rviz2',
                     node_name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[{'robot_description': parameters['robot_description']}])

    # Publish base link TF
    static_tf = Node(package='tf2_ros',
                     node_executable='static_transform_publisher',
                     node_name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'panda_link0'])

    # Fake joint driver
    fake_joint_driver_node = Node(package='fake_joint_driver',
                                  node_executable='fake_joint_driver_node',
                                  # TODO(JafarAbdi): Why this launch the two nodes (controller manager and the fake joint driver) with the same name!
                                  # node_name='fake_joint_driver_node',
                                  parameters=[os.path.join(get_package_share_directory("run_moveit_cpp"), "config", "panda_controllers.yaml"),
                                              os.path.join(get_package_share_directory("run_moveit_cpp"), "config", "start_positions.yaml"),
                                              {'robot_description': parameters['robot_description']}]
                                  )

    return LaunchDescription([ static_tf, rviz_node, run_moveit_cpp_node, fake_joint_driver_node ])
