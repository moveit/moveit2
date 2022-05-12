from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


class DeclareBooleanLaunchArg(DeclareLaunchArgument):
    """This launch action declares a launch argument with true and false as the only valid values"""

    def __init__(self, name, *, default_value=False, description=None, **kwargs):
        super().__init__(
            name,
            default_value=str(default_value),
            description=description,
            choices=["true", "false", "True", "False"],
            **kwargs,
        )


def add_debuggable_node(
    ld,
    package,
    executable,
    condition_name="debug",
    commands_file=None,
    extra_debug_args=None,
    **kwargs,
):
    """Adds two versions of a Node to the launch description, one with gdb debugging, controlled by a launch config"""
    standard_node = Node(
        package=package,
        executable=executable,
        condition=UnlessCondition(LaunchConfiguration(condition_name)),
        **kwargs,
    )
    ld.add_action(standard_node)

    if commands_file:
        dash_x_arg = f"-x {commands_file} "
    else:
        dash_x_arg = ""
    prefix = [f"gdb {dash_x_arg}--ex run --args"]

    if "arguments" in kwargs:
        arguments = kwargs["arguments"]
        if extra_debug_args:
            arguments += extra_debug_args
        del kwargs["arguments"]
    else:
        arguments = None

    debug_node = Node(
        package=package,
        executable=executable,
        condition=IfCondition(LaunchConfiguration(condition_name)),
        prefix=prefix,
        arguments=arguments,
        **kwargs,
    )
    ld.add_action(debug_node)
