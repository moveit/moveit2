import os
import launch
import launch_ros
import launch_testing
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_servo_test_description(
    *args,
    gtest_name: launch.some_substitutions_type.SomeSubstitutionsType,
    start_position_path: launch.some_substitutions_type.SomeSubstitutionsType = ""
):

    # Get parameters using the demo config file
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/panda_simulated_config.yaml")
        .to_dict()
    }

    # Get URDF and SRDF
    if start_position_path:
        initial_positions_file = os.path.join(
            os.path.dirname(__file__), start_position_path
        )
        robot_description_mappings = {"initial_positions_file": initial_positions_file}
    else:
        robot_description_mappings = None

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro", mappings=robot_description_mappings
        )
        .to_moveit_configs()
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # Component nodes for tf and Servo
    test_container = launch_ros.actions.ComposableNodeContainer(
        name="test_servo_integration_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"/child_frame_id": "panda_link0", "/frame_id": "world"}],
            ),
        ],
        output="screen",
    )

    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.joint_limits,
            moveit_config.robot_description_kinematics,
        ],
    )

    # NOTE: The smoothing plugin doesn't seem to be accessible from containers
    # servo_container = ComposableNodeContainer(
    #     name="servo_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::ServoNode",
    #             name="servo_node",
    #             parameters=[
    #                 servo_params,
    #                 robot_description,
    #                 robot_description_semantic,
    #                 joint_limits_yaml,
    #             ],
    #         ),
    #     ],
    #     output="screen",
    # )

    # Unknown how to set timeout
    # https://github.com/ros2/launch/issues/466
    servo_gtest = launch_ros.actions.Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [launch.substitutions.LaunchConfiguration("test_binary_dir"), gtest_name]
        ),
        parameters=[servo_params],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="test_binary_dir",
                description="Binary directory of package "
                "containing test executables",
            ),
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            servo_node,
            test_container,
            launch.actions.TimerAction(period=2.0, actions=[servo_gtest]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        "servo_node": servo_node,
        "test_container": test_container,
        "servo_gtest": servo_gtest,
        "ros2_control_node": ros2_control_node,
    }
