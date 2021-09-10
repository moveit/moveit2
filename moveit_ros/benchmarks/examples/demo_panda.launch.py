from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from parameter_builder import ParameterBuilder


def generate_launch_description():

    moveit_ros_benchmarks_config = (
        ParameterBuilder("moveit_ros_benchmarks")
        .yaml(
            parameter_namespace="benchmark_config",
            file_path="demo1.yaml",
        )
        .to_dict()
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines()
        .to_moveit_configs()
    )

    # moveit_ros_benchmark demo executable
    moveit_ros_benchmarks_node = Node(
        name="moveit_run_benchmark",
        package="moveit_ros_benchmarks",
        # prefix='xterm -e gdb --ex=run --args',
        executable="moveit_run_benchmark",
        output="screen",
        parameters=[
            moveit_ros_benchmarks_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
    )

    return LaunchDescription([moveit_ros_benchmarks_node, mongodb_server_node])
