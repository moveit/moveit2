from pathlib import Path
from typing import Optional, List
import logging
import xacro
from ament_index_python.packages import get_package_share_directory

from parameter_builder import ParameterBuilder, load_yaml


class MoveItConfigs(object):
    def __setattr__(self, name, value):
        if hasattr(self, name):
            object.__setattr__(self, name, value)
        else:
            raise AttributeError(
                f"Cannot set name {name} on object of type {self.__class__.__name__}"
            )

    robot_description = None
    robot_description_semantic = None
    robot_description_planning = None
    robot_description_kinematics = None
    planning_pipelines = None
    trajectory_execution = None
    planning_scene_monitor = None
    move_group_capabilities = None
    move_group = None
    joint_limits = None


class MoveItConfigsBuilder(ParameterBuilder):
    __moveit_configs = MoveItConfigs()
    __robot_name: str
    __urdf_package: Path
    __urdf_filename: str
    __srdf_filename: str
    __robot_description: str

    # Look-up for robot_name_moveit_config package
    def __init__(self, robot_name: str, robot_description="robot_description"):
        super().__init__(robot_name + "_moveit_config")
        self.__robot_name = robot_name
        setup_assistant_file = self._package_path / ".setup_assistant"
        if not setup_assistant_file.exists():
            # TODO: Should this throw an exception .?
            logging.warning(
                f"\x1b[33;21mPackage `{self._package_path}` doesn't have `.setup_assistant` file "
                f"-- using config/{robot_name}.urdf and config/{robot_name}.srdf\x1b[0m"
            )
            self.__urdf_package = self._package_path
            self.__urdf_filename = "config/" + self.__robot_name + ".urdf"
            self.__srdf_filename = "config/" + self.__robot_name + ".srdf"
        else:
            setup_assistant_yaml = load_yaml(setup_assistant_file)
            self.__urdf_package = Path(
                get_package_share_directory(
                    setup_assistant_yaml["moveit_setup_assistant_config"]["URDF"][
                        "package"
                    ]
                )
            )
            self.__urdf_filename = setup_assistant_yaml[
                "moveit_setup_assistant_config"
            ]["URDF"]["relative_path"]
            self.__srdf_filename = setup_assistant_yaml[
                "moveit_setup_assistant_config"
            ]["SRDF"]["relative_path"]
        self.__robot_description = robot_description

    def robot_description(self, file_name: Optional[str] = None, mappings: dict = None):
        if file_name is None:
            robot_description_file = xacro.process_file(
                self.__urdf_package / self.__urdf_filename, mappings=mappings
            )
        else:
            robot_description_file = xacro.process_file(
                self._package_path / "config" / file_name, mappings=mappings
            )
        self.__moveit_configs.robot_description = {
            self.__robot_description: robot_description_file.toxml()
        }
        return self

    def robot_description_semantic(
        self, file_name: Optional[str] = None, mappings: dict = None
    ):
        robot_description_semantic = xacro.process_file(
            self._package_path
            / (("config/" + file_name) if file_name else self.__srdf_filename),
            mappings=mappings,
        )
        self.__moveit_configs.robot_description_semantic = {
            self.__robot_description + "_semantic": robot_description_semantic.toxml()
        }
        return self

    def robot_description_kinematics(self, file_name: Optional[str] = None):
        self.__moveit_configs.robot_description_kinematics = {
            self.__robot_description
            + "_kinematics": load_yaml(
                self._package_path / "config" / (file_name or "kinematics.yaml")
            )
        }
        return self

    def joint_limits(self, file_name: Optional[str] = None):
        self.__moveit_configs.joint_limits = {
            self.__robot_description
            + "_planning": load_yaml(
                self._package_path / "config" / (file_name or "joint_limits.yaml")
            )
        }
        return self

    def trajectory_execution(
        self,
        file_name: Optional[str] = None,
        moveit_manage_controllers: bool = True,
    ):
        self.__moveit_configs.trajectory_execution = {
            "moveit_manage_controllers": moveit_manage_controllers,
        }
        self.__moveit_configs.trajectory_execution.update(
            load_yaml(
                self._package_path
                / "config"
                / (file_name or self.__robot_name + "_controllers.yaml")
            )
        )
        return self

    def planning_scene_monitor(
        self,
        publish_planning_scene: bool = True,
        publish_geometry_updates: bool = True,
        publish_state_updates: bool = True,
        publish_transforms_updates: bool = True,
    ):
        self.__moveit_configs.planning_scene_monitor = {
            # TODO: Fix parameter namespace upstream -- see planning_scene_monitor.cpp:262
            # "planning_scene_monitor": {
            "publish_planning_scene": publish_planning_scene,
            "publish_geometry_updates": publish_geometry_updates,
            "publish_state_updates": publish_state_updates,
            "publish_transforms_updates": publish_transforms_updates,
            # }
        }
        return self

    def planning_pipelines(
        self, default_planning_pipeline: str = None, pipelines: List[str] = None
    ):
        if pipelines is None:
            pipelines = ["ompl"]
            default_planning_pipeline = pipelines[0]
        elif default_planning_pipeline not in pipelines:
            raise RuntimeError(
                f"default_planning_pipeline: `{default_planning_pipeline}` doesn't name any of the input pipelines `{','.join(pipelines)}`"
            )
        self.__moveit_configs.planning_pipelines = {
            "planning_pipelines": pipelines,
            "default_planning_pipeline": default_planning_pipeline,
        }
        for pipeline in pipelines:
            self.__moveit_configs.planning_pipelines[pipeline] = load_yaml(
                self._package_path / "config" / (pipeline + "_planning.yaml")
            )
        return self

    def moveit_configs(self):
        return self.__moveit_configs

    def to_dict(self):
        parameters = self._parameters
        parameters.update(self.__moveit_configs.robot_description)
        parameters.update(self.__moveit_configs.robot_description_semantic)
        parameters.update(self.__moveit_configs.robot_description_kinematics)
        parameters.update(self.__moveit_configs.planning_pipelines)
        parameters.update(self.__moveit_configs.trajectory_execution)
        parameters.update(self.__moveit_configs.planning_scene_monitor)
        parameters.update(self.__moveit_configs.joint_limits)
        return parameters


# USAGE
# moveit_config = (
#     MoveItConfigsBuilder("ROBOT_NAME")
#         .robot_description()
#         .robot_description_semantic()
#         .robot_description_kinematics()
#         .joint_limits()
#         .trajectory_execution(file_name="panda_gripper_controllers.yaml")
#         .planning_scene_monitor()
#         .planning_pipelines()
#         .moveit_configs()
# )
