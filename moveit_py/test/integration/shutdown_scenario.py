"""Isolated MoveItPy lifecycle scenarios: a native crash must fail the parent test."""

import gc
import importlib
import os
from pathlib import Path
import sys
import time
import types

import rclpy
from rcl_interfaces.srv import GetParameters
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.srv import GetPlanningScene

module_name = os.environ.get("MOVEIT_SHUTDOWN_MODULE", "test_moveit.planning")
if module_name.startswith("test_moveit."):
    # The planning bindings import moveit.utils. Keep that import on the source
    # package and share the built core, rather than also registering installed types.
    core = importlib.import_module("test_moveit.core")
    package = types.ModuleType("moveit")
    package.__path__ = [str(Path(__file__).resolve().parents[2] / "moveit")]
    sys.modules["moveit"] = package
    sys.modules["moveit.core"] = core

MoveItPy = importlib.import_module(module_name).MoveItPy


def configuration():
    config = (
        MoveItConfigsBuilder(
            "panda", package_name="moveit_resources_panda_moveit_config"
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
        .to_dict()
    )
    config["planning_scene_monitor_options"] = {
        "name": "planning_scene_monitor",
        "robot_description": "robot_description",
        "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": "/attached_collision_object",
        "publish_planning_scene_topic": "/publish_planning_scene",
        "monitored_planning_scene_topic": "/monitored_planning_scene",
        "wait_for_initial_state_timeout": 0.0,
    }
    config["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    config["plan_request_params"] = {
        "planning_attempts": 1,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "planning_time": 1.0,
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
    }
    return config


def create(name, config=None):
    config = configuration() if config is None else config
    config["shutdown_test_id"] = name
    return MoveItPy(
        node_name=name,
        config_dict=config,
        remappings={"get_planning_scene": f"/{name}/get_planning_scene"},
    )


parameter_clients = {}


def request_parameters(observer, name, expect_response=True):
    # Reuse the established connection when checking that shutdown stops replies.
    if name not in parameter_clients:
        parameter_clients[name] = observer.create_client(
            GetParameters, f"/{name}/get_parameters"
        )
    client = parameter_clients[name]
    assert client.wait_for_service(timeout_sec=5.0)
    deadline = time.monotonic() + 5.0
    while True:
        request = GetParameters.Request()
        request.names = ["shutdown_test_id"]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            observer, future, timeout_sec=0.5 if expect_response else 1.0
        )
        if expect_response:
            if future.done():
                assert future.result() is not None
                assert future.result().values[0].string_value == name
                return
            # Fast DDS can advertise a service before the first request reaches it.
            # This read-only query is safe to retry while discovery completes.
            client.remove_pending_request(future)
            assert time.monotonic() < deadline, "Parameter service did not reply"
        else:
            assert not future.done(), "The stopped executor still answered a request"
            client.remove_pending_request(future)
            return


def request_planning_scene(observer, name):
    client = observer.create_client(GetPlanningScene, f"/{name}/get_planning_scene")
    try:
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(GetPlanningScene.Request())
        rclpy.spin_until_future_complete(observer, future, timeout_sec=5.0)
        assert future.done() and future.result() is not None
    finally:
        observer.destroy_client(client)


def main(scenario):
    rclpy.init()
    observer = rclpy.create_node("shutdown_regression_observer")
    try:
        if scenario == "explicit":
            robot = create("shutdown_explicit")
            request_parameters(observer, "shutdown_explicit")
            component = robot.get_planning_component("panda_arm")
            robot.shutdown()
            robot.shutdown()
            request_parameters(observer, "shutdown_explicit", expect_response=False)
            del robot
            # The component retains its owner after explicit shutdown.
            del component
            gc.collect()
        elif scenario == "implicit":
            robot = create("shutdown_implicit")
            request_parameters(observer, "shutdown_implicit")
            del robot
            gc.collect()
        elif scenario == "repeated":
            for index in range(5):
                robot = create(f"shutdown_repeated_{index}")
                request_parameters(observer, f"shutdown_repeated_{index}")
                robot.shutdown()
                del robot
                gc.collect()
        elif scenario == "independent":
            first = create("shutdown_first")
            second = create("shutdown_second")
            request_parameters(observer, "shutdown_first")
            request_parameters(observer, "shutdown_second")
            request_planning_scene(observer, "shutdown_first")
            request_planning_scene(observer, "shutdown_second")
            first.shutdown()
            del first
            gc.collect()
            # Stopping one executor must not shut down the other's ROS context.
            request_parameters(observer, "shutdown_second")
            second.shutdown()
            del second
            gc.collect()
        elif scenario == "construction_failure":
            config = configuration()
            config["planning_pipelines"] = {"pipeline_names": ["missing_pipeline"]}
            try:
                create("shutdown_invalid", config)
            except RuntimeError:
                pass
            else:
                raise AssertionError("Invalid pipeline should fail construction")
            robot = create("shutdown_recovered")
            request_parameters(observer, "shutdown_recovered")
            robot.shutdown()
            del robot
            gc.collect()
        else:
            raise AssertionError(scenario)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    print(f"SHUTDOWN_OK {scenario}", flush=True)


if __name__ == "__main__":
    main(sys.argv[1])
