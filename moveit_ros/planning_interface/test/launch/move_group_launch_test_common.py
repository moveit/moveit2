import os
import launch
import launch_ros
import launch_testing
from launch.actions import ExecuteProcess, TimerAction
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_move_group_test_description(*args, gtest_name: SomeSubstitutionsType):
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    moveit_config.planning_pipelines["ompl"]["panda_arm"][
        "enforce_constrained_state_space"
    ] = True
    moveit_config.planning_pipelines["ompl"]["panda_arm"][
        "projection_evaluator"
    ] = "joints(panda_joint1,panda_joint2)"

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

    # test executable
    ompl_constraint_test = launch_ros.actions.Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration("test_binary_dir"), gtest_name]
        ),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            run_move_group_node,
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
            TimerAction(period=2.0, actions=[ompl_constraint_test]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "run_move_group_node": run_move_group_node,
        "static_tf": static_tf,
        "robot_state_publisher": robot_state_publisher,
        "ros2_control_node": ros2_control_node,
        "ompl_constraint_test": ompl_constraint_test,
    }
