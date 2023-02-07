from pathlib import Path

from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    ThisLaunchFileDir,
)

from moveit_configs_utils.substitutions import Xacro


def test_xacro_subst():
    subst = parse_substitution(r"$(xacro '$(dirname)/robot.urdf.xacro')")
    context = LaunchContext()
    context.extend_locals({"current_launch_file_directory": str(Path(__file__).parent)})
    assert len(subst) == 1
    assert isinstance(subst[0].file_path[0], ThisLaunchFileDir)
    assert isinstance(subst[0].file_path[1], TextSubstitution)
    assert isinstance(subst[0], Xacro)
    subst[0].perform(context)


def test_xacro_no_mappings():
    xacro = Xacro([ThisLaunchFileDir(), "robot.urdf.xacro"])
    context = LaunchContext()
    context.extend_locals(
        {"current_launch_file_directory": str(Path(__file__).parent) + "/"}
    )
    assert xacro.perform(context)


def test_xacro_with_mappings():
    xacro = Xacro(
        PathJoinSubstitution([ThisLaunchFileDir(), "robot.urdf.xacro"]),
        mappings={
            ("test_", LaunchConfiguration("myrobot")): [
                LaunchConfiguration("myrobot"),
                "sadf",
            ],
            "number": LaunchConfiguration("number"),
            "postfix": LaunchConfiguration("postfix"),
        },
    )
    context = LaunchContext()
    context.extend_locals(
        {"current_launch_file_directory": str(Path(__file__).parent) + "/"}
    )
    SetLaunchConfiguration("myrobot", "robot").visit(context)
    SetLaunchConfiguration("number", "2000").visit(context)
    SetLaunchConfiguration("postfix", "mypostfix").visit(context)
    assert xacro.perform(context)
