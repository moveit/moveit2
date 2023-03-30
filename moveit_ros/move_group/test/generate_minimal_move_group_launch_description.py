#!/usr/bin/env python3

# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import os
import xacro
import yaml

import launch
import launch_testing
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_robot_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    return robot_description


def get_robot_description_semantic():
    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    return robot_description_semantic


def get_ompl_planning_pipeline():
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    return planning_pipelines_config


def generate_minimal_move_group_launch_description():
    """:return: A LaunchDescription for the integration test scenario."""
    robot_description = get_robot_description()

    robot_description_semantic = get_robot_description_semantic()

    ompl_planning_pipeline = get_ompl_planning_pipeline()

    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline["ompl"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        ompl_planning_pipeline,
        moveit_controllers,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            move_group_node,
            launch.actions.TimerAction(
                period=1.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )
