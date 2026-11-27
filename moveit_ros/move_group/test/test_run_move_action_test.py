# Copyright (c) 2026, MoveIt Contributors
# SPDX-License-Identifier: BSD-3-Clause

import os
from pathlib import Path
import signal
import subprocess
import time
import sys
import tempfile
import unittest
from unittest.mock import patch

from run_move_action_test import isolated_environment, run


class TestLauncher(unittest.TestCase):
    def test_success(self):
        self.assertEqual(run([sys.executable, "-c", "pass"], 5), 0)

    def test_nonzero_exit(self):
        self.assertEqual(run([sys.executable, "-c", "raise SystemExit(7)"], 5), 7)

    def test_signal(self):
        command = "import os, signal; os.kill(os.getpid(), signal.SIGTERM)"
        self.assertEqual(run([sys.executable, "-c", command], 5), 128 + signal.SIGTERM)

    def test_reserved_domains_exclude_inherited_and_each_other(self):
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "1"}):
            with isolated_environment() as first, isolated_environment() as second:
                self.assertNotEqual(first["ROS_DOMAIN_ID"], "1")
                self.assertNotEqual(second["ROS_DOMAIN_ID"], "1")
                self.assertNotEqual(first["ROS_DOMAIN_ID"], second["ROS_DOMAIN_ID"])

    @unittest.skipUnless(sys.platform.startswith("linux"), "Checks Linux process state")
    def test_interrupt_cleans_up_child(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "worker.pid"
            command = (
                "import os, pathlib, sys, time; "
                "pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(30)"
            )
            runner = Path(__file__).with_name("run_move_action_test.py")
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(runner),
                    "--timeout",
                    "10",
                    sys.executable,
                    "-c",
                    command,
                    str(pid_file),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 5
                while not pid_file.exists() and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue(pid_file.exists())
                process.send_signal(signal.SIGTERM)
                output, error = process.communicate(timeout=5)
                self.assertNotEqual(process.returncode, 0)
                self.assertIn("SIGTERM", error)
                self.assertFalse(Path(f"/proc/{pid_file.read_text()}").exists(), output)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    @unittest.skipUnless(sys.platform.startswith("linux"), "Checks Linux process state")
    def test_timeout_kills_process_group(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "descendant.pid"
            command = (
                "import pathlib, subprocess, sys; "
                "p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)']); "
                "pathlib.Path(sys.argv[1]).write_text(str(p.pid)); p.wait()"
            )
            self.assertEqual(
                run([sys.executable, "-c", command, str(pid_file)], 1), 124
            )
            descendant = int(pid_file.read_text())
            status = Path(f"/proc/{descendant}/stat")
            if status.exists():
                # A killed orphan may await reaping by the container's init.
                self.assertEqual(status.read_text().split()[2], "Z")


if __name__ == "__main__":
    unittest.main()
