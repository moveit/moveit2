from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # moveit_setup_assistant executable
    moveit_setup_assistant = Node(
        name="moveit_setup_assistant",
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        output="screen",
    )

    return LaunchDescription([moveit_setup_assistant])
