import os
import sys
import threading
import time
import unittest

from test_moveit.core.collision_detection import CollisionRequest, CollisionResult
from test_moveit.core.planning_scene import PlanningScene
from test_moveit.core.robot_model import RobotModel
from test_moveit.core.robot_state import RobotState

dir_path = os.path.dirname(os.path.realpath(__file__))
URDF_FILE = "{}/fixtures/panda.urdf".format(dir_path)
SRDF_FILE = "{}/fixtures/panda.srdf".format(dir_path)

GROUP = "panda_arm"


def get_robot_model():
    """Helper function that returns a RobotModel instance."""
    return RobotModel(urdf_xml_path=URDF_FILE, srdf_xml_path=SRDF_FILE)


def count_ticks_while(call, duration=0.5):
    """
    Run call() in a loop for duration seconds and count how often another Python thread gets to run meanwhile.

    The switch interval is raised far above duration, so the interpreter does not hand the GIL to the other
    thread on its own: the other thread only runs while call() releases the GIL.
    """
    stop = threading.Event()
    ticks = 0

    def tick():
        nonlocal ticks
        while not stop.is_set():
            ticks += 1
            time.sleep(0.0001)  # releases the GIL, so call() can take it back

    old_interval = sys.getswitchinterval()
    sys.setswitchinterval(30.0)
    ticker = threading.Thread(target=tick, daemon=True)
    try:
        ticker.start()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            call()
        result = ticks
    finally:
        stop.set()
        sys.setswitchinterval(old_interval)
        ticker.join()
    return result


class TestPlanningSceneGilRelease(unittest.TestCase):
    """The collision checks release the GIL, so other Python threads keep running during bulk checks."""

    @classmethod
    def setUpClass(cls):
        cls.robot_model = get_robot_model()

    def setUp(self):
        self.scene = PlanningScene(self.robot_model)
        self.state = RobotState(self.robot_model)
        self.state.set_to_default_values()
        self.state.update()

    def assert_releases_gil(self, call):
        call()  # warm up, so the first call's setup cost is not measured
        ticks = count_ticks_while(call)
        # With the GIL held for the whole loop the other thread gets at most a tick or two.
        self.assertGreater(ticks, 10)

    def test_is_state_valid(self):
        self.assert_releases_gil(lambda: self.scene.is_state_valid(self.state, GROUP))

    def test_is_state_colliding_current_state(self):
        self.assert_releases_gil(lambda: self.scene.is_state_colliding(GROUP))

    def test_is_state_colliding(self):
        self.assert_releases_gil(
            lambda: self.scene.is_state_colliding(self.state, GROUP)
        )

    def test_check_collision(self):
        self.assert_releases_gil(
            lambda: self.scene.check_collision(CollisionRequest(), CollisionResult())
        )

    def test_check_collision_state(self):
        self.assert_releases_gil(
            lambda: self.scene.check_collision(
                CollisionRequest(), CollisionResult(), self.state
            )
        )

    def test_check_self_collision_state(self):
        self.assert_releases_gil(
            lambda: self.scene.check_self_collision(
                CollisionRequest(), CollisionResult(), self.state
            )
        )


if __name__ == "__main__":
    unittest.main()
