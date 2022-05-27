"""Simplify loading moveit config parameters.

This module provides builder-pattern based class to simplify loading moveit related parameters found in
robot_moveit_config package generated by moveit setup assistant.

By default it expects the following structure for the moveit configs package

robot_name_moveit_config/
    .setup_assistant -> Used to retrieve information about the SRDF file and
                        the URDF file used when generating the package
    config/
        kinematics.yaml -> IK solver's parameters
        joint_limits.yaml -> Overriding position/velocity/acceleration limits from the URDF file
        moveit_cpp.yaml -> MoveItCpp related parameters
        *_planning.yaml -> planning pipelines parameters
        cartesian_limits.yaml -> Pilz planner parameters
        # TODO(JafarAbdi): Check to see if this is a good default value
        robot_name_controllers.yaml -> trajectory execution manager's parameters
        ...

Example:
    moveit_configs = MoveItConfigsBuilder("robot_name").to_moveit_configs()
    ...
    moveit_configs.package_path
    moveit_configs.robot_description
    moveit_configs.robot_description_semantic
    moveit_configs.robot_description_kinematics
    moveit_configs.planning_pipelines
    moveit_configs.trajectory_execution
    moveit_configs.planning_scene_monitor
    moveit_configs.move_group_capabilities
    moveit_configs.joint_limits
    moveit_configs.moveit_cpp
    moveit_configs.cartesian_limits
    # Or to get all the parameters as a dictionary
    moveit_configs.to_dict()

Each function in MoveItConfigsBuilder has a file_path as an argument which is used to override the default
path for the file

Example:
    moveit_configs = MoveItConfigsBuilder("robot_name")
                    # Relative to robot_name_moveit_configs
                    .robot_description_semantic(Path("my_config") / "my_file.srdf")
                    .to_moveit_configs()
    # Or
    moveit_configs = MoveItConfigsBuilder("robot_name")
                    # Absolute path to robot_name_moveit_config
                    .robot_description_semantic(Path.home() / "my_config" / "new_file.srdf")
                    .to_moveit_configs()
"""

from pathlib import Path
from typing import Optional, List, Union
import logging
import xacro

import re
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder, load_yaml, load_xacro
from jinja2 import Template
from urdf_parser_py.urdf import URDF
from xml.dom.minidom import Document, parseString

moveit_configs_utils_path = Path(get_package_share_directory("moveit_configs_utils"))


def generate_fake_system_description(
    robot_description: Document,
    initial_position={},
) -> Document:
    """Generate ros2 control fake system description from urdf file"""
    robot = URDF.from_xml_string(robot_description.toxml())
    template = Template(
        """
    <ros2_control name="{{name}}" type="system">
        <hardware>
            <plugin>fake_components/GenericSystem</plugin>
        </hardware>
        {% for joint in joints -%}
        {% if joint.type in ["revolute", "continuous", "prismatic"] -%}
        <joint name="{{joint.name}}">
            {% if joint.mimic -%}
            <param name="mimic">{{joint.mimic.joint}}</param>
            {% if joint.mimic.multiplier -%}
            <param name="multiplier">{{joint.mimic.multiplier}}</param>
            {%- endif %}
            {%- endif %}
            <command_interface name="position"/>
            <state_interface name="position">
              <param name="initial_value">{{initial_position.get(joint.name, 0.0)}}</param>
            </state_interface>
            <state_interface name="velocity">
              <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        {%- endif %}
        {%- endfor %}
    </ros2_control>
    """
    )
    return parseString(
        template.render(
            joints=robot.joints,
            name=f"{robot.name.capitalize()}FakeSystem",
            initial_position=initial_position or {},
        )
    )


def get_pattern_matches(folder, pattern):
    """Given all the files in the folder, find those that match the pattern.

    If there are groups defined, the groups are returned. Otherwise the path to the matches are returned.
    """
    matches = []
    if not folder.exists():
        return matches
    for child in folder.iterdir():
        if not child.is_file():
            continue
        m = pattern.search(child.name)
        if m:
            groups = m.groups()
            if groups:
                matches.append(groups[0])
            else:
                matches.append(child)
    return matches


class MoveItConfigs(object):
    """Class containing MoveIt related parameters."""

    __slots__ = [
        "__package_path",
        "__robot_description",
        "__robot_description_semantic",
        "__robot_description_kinematics",
        "__planning_pipelines",
        "__trajectory_execution",
        "__planning_scene_monitor",
        "__move_group_capabilities",
        "__joint_limits",
        "__moveit_cpp",
        "__cartesian_limits",
    ]

    def __init__(self):
        # A pathlib Path to the moveit config package
        self.package_path = None
        # A dictionary that has the contents of the URDF file.
        self.robot_description = {}
        # A dictionary that has the contents of the SRDF file.
        self.robot_description_semantic = {}
        # A dictionary IK solver specific parameters.
        self.robot_description_kinematics = {}
        # A dictionary that contains the planning pipelines parameters.
        self.planning_pipelines = {}
        # A dictionary contains parameters for trajectory execution & moveit controller managers.
        self.trajectory_execution = {}
        # A dictionary that have the planning scene monitor's parameters.
        self.planning_scene_monitor = {}
        # A dictionary containing move_group's non-default capabilities.
        self.move_group_capabilities = {}
        # A dictionary containing the overridden position/velocity/acceleration limits.
        self.joint_limits = {}
        # A dictionary containing MoveItCpp related parameters.
        self.moveit_cpp = {}
        # A dictionary containing the cartesian limits for the Pilz planner.
        self.cartesian_limits = {}

    @property
    def package_path(self):
        return self.__package_path

    @package_path.setter
    def package_path(self, value):
        self.__package_path = value

    @property
    def robot_description(self):
        return self.__robot_description

    @robot_description.setter
    def robot_description(self, value):
        self.__robot_description = value

    @property
    def robot_description_semantic(self):
        return self.__robot_description_semantic

    @robot_description_semantic.setter
    def robot_description_semantic(self, value):
        self.__robot_description_semantic = value

    @property
    def robot_description_kinematics(self):
        return self.__robot_description_kinematics

    @robot_description_kinematics.setter
    def robot_description_kinematics(self, value):
        self.__robot_description_kinematics = value

    @property
    def planning_pipelines(self):
        return self.__planning_pipelines

    @planning_pipelines.setter
    def planning_pipelines(self, value):
        self.__planning_pipelines = value

    @property
    def trajectory_execution(self):
        return self.__trajectory_execution

    @trajectory_execution.setter
    def trajectory_execution(self, value):
        self.__trajectory_execution = value

    @property
    def planning_scene_monitor(self):
        return self.__planning_scene_monitor

    @planning_scene_monitor.setter
    def planning_scene_monitor(self, value):
        self.__planning_scene_monitor = value

    @property
    def move_group_capabilities(self):
        return self.__move_group_capabilities

    @move_group_capabilities.setter
    def move_group_capabilities(self, value):
        self.__move_group_capabilities = value

    @property
    def joint_limits(self):
        return self.__joint_limits

    @joint_limits.setter
    def joint_limits(self, value):
        self.__joint_limits = value

    @property
    def moveit_cpp(self):
        return self.__moveit_cpp

    @moveit_cpp.setter
    def moveit_cpp(self, value):
        self.__moveit_cpp = value

    @property
    def cartesian_limits(self):
        return self.__cartesian_limits

    @cartesian_limits.setter
    def cartesian_limits(self, value):
        self.__cartesian_limits = value

    def to_dict(self):
        parameters = {}
        parameters.update(self.robot_description)
        parameters.update(self.robot_description_semantic)
        parameters.update(self.robot_description_kinematics)
        parameters.update(self.planning_pipelines)
        parameters.update(self.trajectory_execution)
        parameters.update(self.planning_scene_monitor)
        parameters.update(self.joint_limits)
        parameters.update(self.moveit_cpp)
        parameters.update(self.cartesian_limits)
        return parameters


class MoveItConfigsBuilder(ParameterBuilder):
    __moveit_configs = MoveItConfigs()
    __robot_name: str
    __urdf_package: Path
    # Relative path of the URDF file w.r.t. __urdf_package
    __urdf_file_path: Path
    # Relative path of the SRDF file  w.r.t. robot_name_moveit_config
    __srdf_file_path: Path
    # String specify the parameter name that the robot description will be loaded to, it will also be used as a prefix
    # for "_planning", "_semantic", and "_kinematics"
    __robot_description: str
    __config_dir_path = Path("config")

    # Look-up for robot_name_moveit_config package
    def __init__(
        self,
        robot_name: str,
        robot_description="robot_description",
        package_name: Optional[str] = None,
    ):
        super().__init__(package_name or (robot_name + "_moveit_config"))
        self.__moveit_configs.package_path = self._package_path
        self.__robot_name = robot_name
        setup_assistant_file = self._package_path / ".setup_assistant"

        self.__urdf_package = None
        self.__urdf_file_path = None
        self.__srdf_file_path = None

        if setup_assistant_file.exists():
            setup_assistant_yaml = load_yaml(setup_assistant_file)
            config = setup_assistant_yaml.get("moveit_setup_assistant_config", {})
            urdf_config = config.get("urdf", config.get("URDF"))
            if urdf_config:
                self.__urdf_package = Path(
                    get_package_share_directory(urdf_config["package"])
                )
                self.__urdf_file_path = Path(urdf_config["relative_path"])

            srdf_config = config.get("srdf", config.get("SRDF"))
            if srdf_config:
                self.__srdf_file_path = Path(srdf_config["relative_path"])

        if not self.__urdf_package or not self.__urdf_file_path:
            logging.warning(
                f"\x1b[33;21mCannot infer URDF from `{self._package_path}`. -- using config/{robot_name}.urdf\x1b[0m"
            )
            self.__urdf_package = self._package_path
            self.__urdf_file_path = self.__config_dir_path / (
                self.__robot_name + ".urdf"
            )

        if not self.__srdf_file_path:
            logging.warning(
                f"\x1b[33;21mCannot infer SRDF from `{self._package_path}`. -- using config/{robot_name}.srdf\x1b[0m"
            )
            self.__srdf_file_path = self.__config_dir_path / (
                self.__robot_name + ".srdf"
            )

        self.__robot_description = robot_description

    def robot_description(
        self,
        file_path: Optional[str] = None,
        mappings: dict = None,
        auto_generate_fake_components: bool = True,
        initial_positions: Union[str, dict] = None,
    ):
        """Load robot description.

        :param file_path: Absolute or relative path to the URDF file (w.r.t. robot_name_moveit_config).
        :param mappings: Mappings to be passed when loading the xacro file.
        :param auto_generate_fake_components: If true moveit_configs_utils will automatically generate the ros2_control description for the fake components
        :param initial_positions: A dictionary of joint_name -> initial_joint_position, only used when ros2_control description is automatically generated
        :return: Instance of MoveItConfigsBuilder with robot_description loaded.
        """
        if file_path is None:
            robot_description_file_path = self.__urdf_package / self.__urdf_file_path
        else:
            robot_description_file_path = self._package_path / file_path
        robot_description = xacro.process_file(
            robot_description_file_path, mappings=mappings
        )
        if auto_generate_fake_components:
            if (
                len(
                    robot_description.documentElement.getElementsByTagName(
                        "ros2_control"
                    )
                )
                != 0
            ):
                raise RuntimeError(
                    f"Can't auto-generate ros2_control fake components description, provided robot_description file '{robot_description_file_path}' already contains 'ros2_control' tag"
                    "Make sure to set auto_generate_fake_components to false when calling robot_description(..., auto_generate_fake_components = false)"
                )
            if type(initial_positions) == str:
                initial_positions = load_yaml(self._package_path / initial_positions)[
                    "initial_positions"
                ]
            if initial_positions is None:
                # For backward compatibility check for config/initial_positions.yaml
                initial_positions_file = (
                    self._package_path / "config" / "initial_positions.yaml"
                )
                if initial_positions_file.exists():
                    initial_positions = load_yaml(initial_positions_file)[
                        "initial_positions"
                    ]
            ros2_control_fake_components_desc = generate_fake_system_description(
                robot_description, initial_positions
            )
            for child in ros2_control_fake_components_desc.childNodes:
                robot_description.documentElement.appendChild(child)
        self.__moveit_configs.robot_description = {
            self.__robot_description: robot_description.toxml()
        }
        return self

    def robot_description_semantic(
        self, file_path: Optional[str] = None, mappings: dict = None
    ):
        """Load semantic robot description.

        :param file_path: Absolute or relative path to the SRDF file (w.r.t. robot_name_moveit_config).
        :param mappings: mappings to be passed when loading the xacro file.
        :return: Instance of MoveItConfigsBuilder with robot_description_semantic loaded.
        """
        self.__moveit_configs.robot_description_semantic = {
            self.__robot_description
            + "_semantic": load_xacro(
                self._package_path / (file_path or self.__srdf_file_path),
                mappings=mappings,
            )
        }
        return self

    def robot_description_kinematics(self, file_path: Optional[str] = None):
        """Load IK solver parameters.

        :param file_path: Absolute or relative path to the kinematics yaml file (w.r.t. robot_name_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_kinematics loaded.
        """
        self.__moveit_configs.robot_description_kinematics = {
            self.__robot_description
            + "_kinematics": load_yaml(
                self._package_path
                / (file_path or self.__config_dir_path / "kinematics.yaml")
            )
        }
        return self

    def joint_limits(self, file_path: Optional[str] = None):
        """Load joint limits overrides.

        :param file_path: Absolute or relative path to the joint limits yaml file (w.r.t. robot_name_moveit_config).
        :return: Instance of MoveItConfigsBuilder with robot_description_planning loaded.
        """
        self.__moveit_configs.joint_limits = {
            self.__robot_description
            + "_planning": load_yaml(
                self._package_path
                / (file_path or self.__config_dir_path / "joint_limits.yaml")
            )
        }
        return self

    def moveit_cpp(self, file_path: Optional[str] = None):
        """Load MoveItCpp parameters.

        :param file_path: Absolute or relative path to the MoveItCpp yaml file (w.r.t. robot_name_moveit_config).
        :return: Instance of MoveItConfigsBuilder with moveit_cpp loaded.
        """
        self.__moveit_configs.moveit_cpp = load_yaml(
            self._package_path
            / (file_path or self.__config_dir_path / "moveit_cpp.yaml")
        )
        return self

    def trajectory_execution(
        self,
        file_path: Optional[str] = None,
        moveit_manage_controllers: bool = True,
    ):
        """Load trajectory execution and moveit controller managers' parameters

        :param file_path: Absolute or relative path to the controllers yaml file (w.r.t. robot_name_moveit_config).
        :param moveit_manage_controllers: Whether trajectory execution manager is allowed to switch controllers' states.
        :return: Instance of MoveItConfigsBuilder with trajectory_execution loaded.
        """
        self.__moveit_configs.trajectory_execution = {
            "moveit_manage_controllers": moveit_manage_controllers,
        }

        # Find the most likely controller params as needed
        if file_path is None:
            config_folder = self._package_path / self.__config_dir_path
            controller_pattern = re.compile("^(.*)_controllers.yaml$")
            possible_names = get_pattern_matches(config_folder, controller_pattern)
            if not possible_names:
                raise RuntimeError(
                    "trajectory_execution: `Parameter file_path is undefined "
                    f"and no matches for {config_folder}/*_controllers.yaml"
                )
            else:
                chosen_name = None
                if len(possible_names) == 1:
                    chosen_name = possible_names[0]
                else:
                    # Try a couple other common names, in order of precedence
                    for name in ["moveit", "moveit2", self.__robot_name]:
                        if name in possible_names:
                            chosen_name = name
                            break
                    else:
                        option_str = "\n - ".join(
                            name + "_controllers.yaml" for name in possible_names
                        )
                        raise RuntimeError(
                            "trajectory_execution: "
                            f"Unable to guess which parameter file to load. Options:\n - {option_str}"
                        )
                file_path = config_folder / (chosen_name + "_controllers.yaml")

        else:
            file_path = self._package_path / file_path

        self.__moveit_configs.trajectory_execution.update(load_yaml(file_path))
        return self

    def planning_scene_monitor(
        self,
        publish_planning_scene: bool = True,
        publish_geometry_updates: bool = True,
        publish_state_updates: bool = True,
        publish_transforms_updates: bool = True,
        publish_robot_description: bool = False,
        publish_robot_description_semantic: bool = False,
    ):
        self.__moveit_configs.planning_scene_monitor = {
            # TODO: Fix parameter namespace upstream -- see planning_scene_monitor.cpp:262
            # "planning_scene_monitor": {
            "publish_planning_scene": publish_planning_scene,
            "publish_geometry_updates": publish_geometry_updates,
            "publish_state_updates": publish_state_updates,
            "publish_transforms_updates": publish_transforms_updates,
            "publish_robot_description": publish_robot_description,
            "publish_robot_description_semantic": publish_robot_description_semantic,
            # }
        }
        return self

    def planning_pipelines(
        self,
        default_planning_pipeline: str = None,
        pipelines: List[str] = None,
        load_all: bool = True,
    ):
        """Load planning pipelines parameters.

        :param default_planning_pipeline: Name of the default planning pipeline.
        :param pipelines: List of the planning pipelines to be loaded.
        :param load_all: Only used if pipelines is None.
                         If true, loads all pipelines defined in config package AND this package.
                         If false, only loads the pipelines defined in config package.
        :return: Instance of MoveItConfigsBuilder with planning_pipelines loaded.
        """
        config_folder = self._package_path / self.__config_dir_path
        default_folder = moveit_configs_utils_path / "default_configs"

        # If no pipelines are specified, search by filename
        if pipelines is None:
            planning_pattern = re.compile("^(.*)_planning.yaml$")
            pipelines = get_pattern_matches(config_folder, planning_pattern)
            if load_all:
                for pipeline in get_pattern_matches(default_folder, planning_pattern):
                    if pipeline not in pipelines:
                        pipelines.append(pipeline)

        # Define default pipeline as needed
        if not default_planning_pipeline:
            if "ompl" in pipelines:
                default_planning_pipeline = "ompl"
            else:
                default_planning_pipeline = pipelines[0]

        if default_planning_pipeline not in pipelines:
            raise RuntimeError(
                f"default_planning_pipeline: `{default_planning_pipeline}` doesn't name any of the input pipelines "
                f"`{','.join(pipelines)}`"
            )
        self.__moveit_configs.planning_pipelines = {
            "planning_pipelines": pipelines,
            "default_planning_pipeline": default_planning_pipeline,
        }
        for pipeline in pipelines:
            parameter_file = config_folder / (pipeline + "_planning.yaml")
            if not parameter_file.exists():
                parameter_file = default_folder / (pipeline + "_planning.yaml")
            self.__moveit_configs.planning_pipelines[pipeline] = load_yaml(
                parameter_file
            )

        # Special rule to add ompl planner_configs
        if "ompl" in self.__moveit_configs.planning_pipelines:
            ompl_config = self.__moveit_configs.planning_pipelines["ompl"]
            if "planner_configs" not in ompl_config:
                ompl_config.update(load_yaml(default_folder / "ompl_defaults.yaml"))

        return self

    def cartesian_limits(self, file_path: Optional[str] = None):
        """Load cartesian limits.

        :param file_path: Absolute or relative path to the cartesian limits file (w.r.t. robot_name_moveit_config).
        :return: Instance of MoveItConfigsBuilder with cartesian_limits loaded.
        """
        self.__moveit_configs.cartesian_limits = {
            self.__robot_description
            + "_planning": load_yaml(
                self._package_path
                / (file_path or self.__config_dir_path / "cartesian_limits.yaml")
            )
        }
        return self

    def to_moveit_configs(self):
        """Get MoveIt configs from robot_name_moveit_config.

        :return: An MoveItConfigs instance with all parameters loaded.
        """
        if not self.__moveit_configs.robot_description:
            self.robot_description()
        if not self.__moveit_configs.robot_description_semantic:
            self.robot_description_semantic()
        if not self.__moveit_configs.robot_description_kinematics:
            self.robot_description_kinematics()
        if not self.__moveit_configs.planning_pipelines:
            self.planning_pipelines()
        if not self.__moveit_configs.trajectory_execution:
            self.trajectory_execution()
        if not self.__moveit_configs.planning_scene_monitor:
            self.planning_scene_monitor()
        if not self.__moveit_configs.joint_limits:
            self.joint_limits()
        # TODO(JafarAbdi): We should have a default moveit_cpp.yaml as port of a moveit config package
        # if not self.__moveit_configs.moveit_cpp:
        #     self.moveit_cpp()
        if not self.__moveit_configs.cartesian_limits:
            self.cartesian_limits()
        return self.__moveit_configs

    def to_dict(self, include_moveit_configs: bool = True):
        """Get loaded parameters from robot_name_moveit_config as a dictionary.

        :param include_moveit_configs: Whether to include the MoveIt config parameters or
                                       only the ones from ParameterBuilder
        :return: Dictionary with all parameters loaded.
        """
        parameters = self._parameters
        if include_moveit_configs:
            parameters.update(self.to_moveit_configs().to_dict())
        return parameters
