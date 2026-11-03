"""Exercise both publishing modes with a real subscriber and scene geometry."""

import os
import subprocess
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
import yaml


@pytest.mark.parametrize("full_scene", [False, True], ids=["world", "scene"])
def test_publish_scene_from_text(tmp_path, full_scene):
    namespace = f"publish_scene_test_{os.getpid()}_{int(full_scene)}"
    params_file = tmp_path / "parameters.yaml"
    params_file.write_text(
        yaml.safe_dump(
            {
                "/**": {
                    "ros__parameters": {
                        "robot_description": (
                            '<robot name="test_robot"><link name="base_link"/></robot>'
                        ),
                        "robot_description_semantic": '<robot name="test_robot"/>',
                    }
                }
            }
        )
    )
    scene_file = tmp_path / "geometry.scene"
    scene_file.write_text(
        "test_scene\n* test_box\n1\nbox\n0.1 0.2 0.3\n" "1 2 3\n0 0 0 1\n1 0 0 1\n.\n"
    )

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("scene_observer", namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    topic = "planning_scene" if full_scene else "planning_scene_world"
    subscription = node.create_subscription(
        PlanningScene if full_scene else PlanningSceneWorld, topic, received.append, 1
    )
    command = [os.environ["PUBLISH_SCENE_FROM_TEXT_EXECUTABLE"]]
    if full_scene:
        command.append("--scene")
    command.extend(
        [
            str(scene_file),
            "--ros-args",
            "--params-file",
            str(params_file),
            "-r",
            f"__ns:=/{namespace}",
        ]
    )

    try:
        with (tmp_path / "publisher.log").open("w+") as output:
            with subprocess.Popen(
                command, stdout=output, stderr=subprocess.STDOUT
            ) as process:
                try:
                    deadline = time.monotonic() + 15.0
                    while (
                        not received
                        and process.poll() is None
                        and time.monotonic() < deadline
                    ):
                        executor.spin_once(timeout_sec=0.1)
                    process.wait(timeout=5.0)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=5.0)
                output.seek(0)
                diagnostics = output.read()
                assert process.returncode == 0, diagnostics
                assert received, f"No message received on {topic}:\n{diagnostics}"

        message = received[0]
        if full_scene:
            assert message.is_diff
        world = message.world if full_scene else message
        assert len(world.collision_objects) == 1
        collision_object = world.collision_objects[0]
        assert collision_object.id == "test_box"
        assert list(collision_object.primitives[0].dimensions) == pytest.approx(
            [0.1, 0.2, 0.3]
        )
    finally:
        executor.shutdown()
        node.destroy_subscription(subscription)
        node.destroy_node()
        context.shutdown()
