from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():

    moveit_ros_benchmarks_config = (
        ParameterBuilder("moveit_ros_benchmarks")
        .yaml(
            parameter_namespace="benchmark_config",
            file_path="demo_panda_predefined_poses.yaml",
        )
        .to_dict()
    )

    moveit_configs = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    # moveit_ros_benchmark demo executable
    moveit_ros_benchmarks_node = Node(
        node_name="moveit_run_benchmark",
        package="moveit_ros_benchmarks",
        #   prefix='xterm -e gdb --ex=run --args',
        node_executable="moveit_combine_predefined_poses_benchmark",
        output="screen",
        parameters=[
            moveit_ros_benchmarks_config,
            moveit_configs,
        ],
    )

    return LaunchDescription([moveit_ros_benchmarks_node])
