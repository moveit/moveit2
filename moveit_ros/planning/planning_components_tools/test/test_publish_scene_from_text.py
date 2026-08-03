import pathlib
import unittest
from threading import Event
from threading import Thread

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing
import launch_testing.asserts
from launch_testing.util import KeepAliveProc
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PlanningSceneWorld
import pytest
import rclpy
from rclpy.node import Node


SCENARIOS = {
    "default": {
        "arguments": [],
        "scene_topic": "test/default/planning_scene",
        "world_topic": "test/default/planning_scene_world",
        "expect_scene": False,
        "expect_world": True,
    },
    "world": {
        "arguments": ["--world"],
        "scene_topic": "test/world/planning_scene",
        "world_topic": "test/world/planning_scene_world",
        "expect_scene": False,
        "expect_world": True,
    },
    "scene": {
        "arguments": ["--scene"],
        "scene_topic": "test/scene/planning_scene",
        "world_topic": "test/scene/planning_scene_world",
        "expect_scene": True,
        "expect_world": False,
    },
    "scene_world": {
        "arguments": ["--scene", "--world"],
        "scene_topic": "test/scene_world/planning_scene",
        "world_topic": "test/scene_world/planning_scene_world",
        "expect_scene": True,
        "expect_world": True,
    },
}


@pytest.mark.launch_test
def generate_test_description():
    rdf_test_data = get_package_share_path("moveit_ros_planning") / "rdf_loader" / "test" / "data"
    robot_description = (rdf_test_data / "kermit.urdf").read_text()
    robot_description_semantic = (rdf_test_data / "kermit.srdf").read_text()
    scene_file = pathlib.Path(__file__).resolve().parent / "data" / "publish_scene.scene"

    processes = {}
    actions = []
    for name, scenario in SCENARIOS.items():
        process = LaunchNode(
            package="moveit_ros_planning",
            executable="moveit_publish_scene_from_text",
            name=f"publish_scene_from_text_{name}",
            arguments=[*scenario["arguments"], str(scene_file)],
            parameters=[
                {
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                }
            ],
            remappings=[
                ("planning_scene", scenario["scene_topic"]),
                ("planning_scene_world", scenario["world_topic"]),
            ],
            output="screen",
        )
        processes[name] = process
        actions.append(process)

    actions.append(KeepAliveProc())
    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions), processes


class TopicObserver(Node):
    def __init__(self):
        super().__init__("publish_scene_from_text_observer")
        self.events = {}
        self.messages = {}
        self.topic_subscriptions = []
        self.spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

    def observe(self, topic, msg_type):
        event = Event()
        self.events[topic] = event
        self.messages[topic] = None

        def callback(msg):
            self.messages[topic] = msg
            event.set()

        self.topic_subscriptions.append(
            self.create_subscription(msg_type, topic, callback, 1)
        )


def assert_collision_object(test_case, collision_object, scenario_name):
    test_case.assertEqual(collision_object.id, "foo", f"Unexpected collision object id for {scenario_name}")
    test_case.assertEqual(len(collision_object.primitives), 1, f"Expected one primitive in published object for {scenario_name}")
    test_case.assertEqual(len(collision_object.meshes), 0, f"Did not expect meshes in published object for {scenario_name}")

    primitive = collision_object.primitives[0]
    test_case.assertEqual(primitive.type, primitive.BOX, f"Expected box primitive for {scenario_name}")
    test_case.assertEqual(list(primitive.dimensions), [0.1, 0.2, 0.3], f"Unexpected box dimensions for {scenario_name}")
    test_case.assertEqual(len(collision_object.primitive_poses), 1, f"Expected one primitive pose in published object for {scenario_name}")


def assert_object_color(test_case, object_color, scenario_name):
    test_case.assertEqual(object_color.id, "foo", f"Unexpected object color id for {scenario_name}")
    test_case.assertAlmostEqual(object_color.color.r, 0.2, msg=f"Unexpected red channel for {scenario_name}")
    test_case.assertAlmostEqual(object_color.color.g, 0.4, msg=f"Unexpected green channel for {scenario_name}")
    test_case.assertAlmostEqual(object_color.color.b, 0.6, msg=f"Unexpected blue channel for {scenario_name}")
    test_case.assertAlmostEqual(object_color.color.a, 0.8, msg=f"Unexpected alpha channel for {scenario_name}")


def wait_for_event_or_shutdown(test_case, event, proc_info, process, scenario_name, topic_name, timeout=5.0):
    elapsed = 0.0
    poll_period = 0.1
    while elapsed < timeout:
        if event.wait(timeout=poll_period):
            return True

        try:
            proc_info.assertWaitForShutdown(process, timeout=0.0)
        except AssertionError:
            elapsed += poll_period
            continue

        test_case.fail(f"{scenario_name} process exited before publishing expected {topic_name}")

    test_case.fail(f"Timed out waiting for expected {topic_name} from {scenario_name}")

class TestPublishSceneFromText(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.observer = TopicObserver()
        for scenario in SCENARIOS.values():
            cls.observer.observe(scenario["scene_topic"], PlanningScene)
            cls.observer.observe(scenario["world_topic"], PlanningSceneWorld)

    @classmethod
    def tearDownClass(cls):
        cls.observer.destroy_node()
        rclpy.shutdown()

    def _assert_scenario(self, proc_info, process, name):
        scenario = SCENARIOS[name]

        if scenario["expect_scene"]:
            scene_received = wait_for_event_or_shutdown(
                self,
                self.observer.events[scenario["scene_topic"]],
                proc_info,
                process,
                name,
                "planning_scene",
            )
        else:
            scene_received = self.observer.events[scenario["scene_topic"]].wait(timeout=0.5)

        if scenario["expect_world"]:
            world_received = wait_for_event_or_shutdown(
                self,
                self.observer.events[scenario["world_topic"]],
                proc_info,
                process,
                name,
                "planning_scene_world",
            )
        else:
            world_received = self.observer.events[scenario["world_topic"]].wait(timeout=0.5)

        self.assertEqual(
            scene_received,
            scenario["expect_scene"],
            f"Unexpected planning scene publication result for {name}",
        )
        self.assertEqual(
            world_received,
            scenario["expect_world"],
            f"Unexpected planning scene world publication result for {name}",
        )

        if scene_received:
            planning_scene = self.observer.messages[scenario["scene_topic"]]
            collision_objects = planning_scene.world.collision_objects
            self.assertEqual(
                len(collision_objects), 1, f"Expected one planning scene collision object for {name}"
            )
            assert_collision_object(self, collision_objects[0], name)
            self.assertEqual(
                len(planning_scene.object_colors), 1, f"Expected one object color entry for {name}"
            )
            assert_object_color(self, planning_scene.object_colors[0], name)
        if world_received:
            collision_objects = self.observer.messages[scenario["world_topic"]].collision_objects
            self.assertEqual(
                len(collision_objects), 1, f"Expected one planning scene world collision object for {name}"
            )
            assert_collision_object(self, collision_objects[0], name)

        proc_info.assertWaitForShutdown(process, timeout=4000.0)

    def test_default_publishes_world(self, proc_info, default):
        self._assert_scenario(proc_info, default, "default")

    def test_world_flag_publishes_world(self, proc_info, world):
        self._assert_scenario(proc_info, world, "world")

    def test_scene_flag_publishes_scene(self, proc_info, scene):
        self._assert_scenario(proc_info, scene, "scene")

    def test_scene_and_world_flags_publish_both(self, proc_info, scene_world):
        self._assert_scenario(proc_info, scene_world, "scene_world")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
