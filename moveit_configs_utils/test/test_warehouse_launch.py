from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch_ros.actions import SetParameter

from moveit_configs_utils import launches


def _execute_parameter_actions(actions, context):
    for action in actions:
        if isinstance(action, (DeclareLaunchArgument, SetParameter)):
            action.execute(context)


def test_generate_warehouse_db_launch_uses_sqlite(monkeypatch, tmp_path):
    node_calls = []
    ros_home = tmp_path / "ros_home"
    config_package_path = tmp_path / "test_moveit_config"
    monkeypatch.setenv("ROS_HOME", str(ros_home))

    def capture_node(**kwargs):
        node_calls.append(kwargs)
        return LogInfo(msg="captured node")

    monkeypatch.setattr(launches, "Node", capture_node)

    moveit_parameters = {
        "robot_description": "<robot name='test'/>",
        "robot_description_semantic": "<robot name='test'/>",
    }
    moveit_config = SimpleNamespace(
        package_path=config_package_path,
        to_dict=lambda: moveit_parameters,
    )

    launch_description = launches.generate_warehouse_db_launch(moveit_config)
    actions = launch_description.entities

    launch_arguments = {
        action.name: action
        for action in actions
        if isinstance(action, DeclareLaunchArgument)
    }
    assert set(launch_arguments) == {
        "moveit_warehouse_database_path",
        "reset",
    }

    set_parameters = [action for action in actions if isinstance(action, SetParameter)]
    assert len(set_parameters) == 2

    context = LaunchContext()
    _execute_parameter_actions(actions, context)

    assert context.launch_configurations["moveit_warehouse_database_path"] == str(
        ros_home / "test_moveit_config_warehouse.sqlite"
    )

    global_parameters = dict(context.launch_configurations["global_params"])
    assert global_parameters == {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": str(ros_home / "test_moveit_config_warehouse.sqlite"),
    }

    custom_database_path = str(tmp_path / "custom.sqlite")
    custom_context = LaunchContext()
    custom_context.launch_configurations["moveit_warehouse_database_path"] = (
        custom_database_path
    )
    _execute_parameter_actions(actions, custom_context)

    custom_parameters = dict(custom_context.launch_configurations["global_params"])
    assert custom_parameters["warehouse_host"] == custom_database_path

    # SQLite requires no separate server, so only the reset node is generated.
    assert len(node_calls) == 1

    reset_node = node_calls[0]
    assert reset_node["package"] == "moveit_ros_warehouse"
    assert reset_node["executable"] == "moveit_init_demo_warehouse"
    assert reset_node["output"] == "screen"
    assert reset_node["parameters"] == [moveit_parameters]
    assert isinstance(reset_node["condition"], IfCondition)

    context.launch_configurations["reset"] = "true"
    assert reset_node["condition"].evaluate(context)

    context.launch_configurations["reset"] = "false"
    assert not reset_node["condition"].evaluate(context)


def test_demo_launch_scopes_warehouse_before_consumers(tmp_path):
    launch_directory = tmp_path / "launch"
    launch_directory.mkdir()

    launch_file_contents = """\
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription()
"""

    for filename in (
        "rsp.launch.py",
        "warehouse_db.launch.py",
        "move_group.launch.py",
        "moveit_rviz.launch.py",
        "spawn_controllers.launch.py",
    ):
        (launch_directory / filename).write_text(launch_file_contents)

    moveit_config = SimpleNamespace(package_path=tmp_path)
    launch_description = launches.generate_demo_launch(moveit_config)

    groups = [
        action
        for action in launch_description.entities
        if isinstance(action, GroupAction)
    ]
    assert len(groups) == 1

    includes = [
        action
        for action in groups[0].get_sub_entities()
        if isinstance(action, IncludeLaunchDescription)
    ]

    context = LaunchContext()
    include_names = []
    for action in includes:
        source = action.launch_description_source
        source.get_launch_description(context)
        include_names.append(Path(source.location).name)

    assert include_names == [
        "warehouse_db.launch.py",
        "move_group.launch.py",
        "moveit_rviz.launch.py",
    ]
