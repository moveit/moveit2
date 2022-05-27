import xacro
from ament_index_python import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder, generate_fake_system_description


def test_moveit_resources_configs():
    for pkg_name in ["moveit_resources_fanuc", "moveit_resources_panda"]:
        try:
            config = MoveItConfigsBuilder(pkg_name)
            assert config.to_dict()
        except RuntimeError as e:
            assert False, f"Default {pkg_name} configuration failed to build: {e}"


def test_panda_with_gripper_config():
    try:
        moveit_config = (
            MoveItConfigsBuilder("moveit_resources_panda")
            .robot_description(file_path="config/panda.urdf.xacro")
            .robot_description_semantic(file_path="config/panda.srdf")
            .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        )
        assert moveit_config.to_dict()
    except RuntimeError as e:
        assert False, f"moveit_resources_panda gripper config failed to build: {e}"


def test_generated_panda_fake_components():
    robot_description = generate_fake_system_description(
        xacro.process_file(
            get_package_share_directory("moveit_resources_panda_description")
            + "/urdf/panda.urdf"
        ),
        initial_position={"panda_joint6": 1.4, "panda_finger_joint1": 0.2},
        initial_velocity={"panda_joint1": -0.1, "panda_joint3": 0.1},
    )
    expected_output = (
        '<?xml version="1.0" ?>\n'
        '<ros2_control name="PandaFakeSystem" type="system">\n'
        "\t<hardware>\n"
        "\t\t<plugin>fake_components/GenericSystem</plugin>\n"
        "\t</hardware>\n"
        '\t<joint name="panda_joint1">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">-0.1</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_joint2">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_joint3">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.1</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_joint4">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_joint5">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_joint6">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">1.4</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_joint7">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_finger_joint1">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.2</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="panda_finger_joint2">\n'
        '\t\t<param name="mimic">panda_finger_joint1</param>\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        "</ros2_control>\n"
    )

    assert expected_output == robot_description.toprettyxml()


def test_generated_fanuc_fake_components():
    robot_description = generate_fake_system_description(
        xacro.process_file(
            get_package_share_directory("moveit_resources_fanuc_description")
            + "/urdf/fanuc.urdf"
        ),
        initial_position={"joint_5": 1.4, "joint_6": 0.2},
        initial_velocity={"joint_1": -0.1, "joint_3": 0.3},
    )
    expected_output = (
        '<?xml version="1.0" ?>\n'
        '<ros2_control name="FanucFakeSystem" type="system">\n'
        "\t<hardware>\n"
        "\t\t<plugin>fake_components/GenericSystem</plugin>\n"
        "\t</hardware>\n"
        '\t<joint name="joint_1">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">-0.1</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="joint_2">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="joint_3">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.3</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="joint_4">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="joint_5">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">1.4</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        '\t<joint name="joint_6">\n'
        '\t\t<command_interface name="position"/>\n'
        '\t\t<state_interface name="position">\n'
        '\t\t\t<param name="initial_value">0.2</param>\n'
        "\t\t</state_interface>\n"
        '\t\t<state_interface name="velocity">\n'
        '\t\t\t<param name="initial_value">0.0</param>\n'
        "\t\t</state_interface>\n"
        "\t</joint>\n"
        "</ros2_control>\n"
    )

    assert expected_output == robot_description.toprettyxml()
