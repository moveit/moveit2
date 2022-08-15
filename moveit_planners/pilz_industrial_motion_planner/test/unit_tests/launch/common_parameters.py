from ament_index_python.packages import get_package_share_directory
import xacro
import yaml
import os

# TODO(henningkayser): Switch to ParameterBuilder once #591 is merged
# from moveit_configs_utils import MoveItConfigsBuilder
# from launch_param_builder import ParameterBuilder


def _load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    # TODO(henningkayser): Switch to ParameterBuilder once #591 is merged
    # return (
    #     ParameterBuilder(package_name)
    #     .yaml(file_path)
    #     .to_dict()
    # )
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


class MoveItConfigs:
    robot_description = {}
    robot_description_semantic = {}
    robot_description_kinematics = {}
    robot_description_planning = {}
    planning_plugin = {}


def load_moveit_config():
    moveit_config_package_name = "moveit_resources_prbt_moveit_config"
    description_package_name = "moveit_resources_prbt_support"
    description_xacro_file = "urdf/prbt.xacro"
    robot_description_semantic_file = "config/prbt.srdf.xacro"
    robot_description_kinematics_file = "config/kinematics.yaml"
    joint_limits_file = "config/joint_limits.yaml"
    pilz_cartesian_limits_file = "config/pilz_cartesian_limits.yaml"
    return (
        MoveItConfigsBuilder(moveit_config_package_name)
        .robot_description(
            file_path=get_package_share_directory(description_package_name)
            + "/"
            + description_xacro_file
        )
        .robot_description_semantic(file_path=robot_description_semantic_file)
        .robot_description_kinematics(file_path=robot_description_kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file)
        .to_moveit_configs()
    )
