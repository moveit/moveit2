"""Check shutdown in child processes so SIGSEGV and hanging threads are failures."""
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize(
    "scenario",
    ["explicit", "implicit", "repeated", "independent", "construction_failure"],
)
def test_moveit_py_shutdown(scenario):
    env = dict(os.environ)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["ROS_DOMAIN_ID"] = str(40 + os.getpid() % 150)
    result = subprocess.run(
        [
            sys.executable,
            "-u",
            str(Path(__file__).with_name("shutdown_scenario.py")),
            scenario,
        ],
        capture_output=True,
        text=True,
        env=env,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"SHUTDOWN_OK {scenario}" in result.stdout
