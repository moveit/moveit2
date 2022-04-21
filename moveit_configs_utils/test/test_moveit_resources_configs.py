from moveit_configs_utils import MoveItConfigsBuilder


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
