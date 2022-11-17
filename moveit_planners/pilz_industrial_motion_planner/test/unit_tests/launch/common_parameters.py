from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def load_yaml(package_name, file_path):
    return ParameterBuilder(package_name).yaml(file_path).to_dict()


def load_moveit_config():
    moveit_config_package_name = "moveit_resources_prbt"
    description_package_name = "moveit_resources_prbt_support"
    description_xacro_file = "urdf/prbt.xacro"
    robot_description_semantic_file = "config/prbt.srdf.xacro"
    robot_description_kinematics_file = "config/kinematics.yaml"
    moveit_controllers_file = "config/prbt_controllers.yaml"
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
        .trajectory_execution(file_path=moveit_controllers_file)
        .joint_limits(file_path=joint_limits_file)
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file)
        .to_moveit_configs()
    )
