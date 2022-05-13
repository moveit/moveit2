from launch import LaunchDescription
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    add_debuggable_node(
        ld,
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
    )

    return ld
