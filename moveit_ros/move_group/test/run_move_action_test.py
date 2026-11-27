# Copyright (c) 2026, MoveIt Contributors
# SPDX-License-Identifier: BSD-3-Clause

"""Run one MoveGroup GTest in a fresh process and a reserved ROS domain."""

import argparse
from contextlib import contextmanager
from itertools import cycle
import os
import signal
import subprocess
import sys

import domain_coordinator


@contextmanager
def isolated_environment():
    # The coordinator reserves domains for cooperating tests. Also avoid the
    # inherited domain, which may belong to a running demo.
    inherited = os.environ.get("ROS_DOMAIN_ID")
    candidates = cycle(str(i) for i in range(1, 101) if str(i) != inherited)
    with domain_coordinator.domain_id(selector=lambda: int(next(candidates))) as domain:
        yield dict(os.environ, ROS_DOMAIN_ID=str(domain))


def interrupt(signum, _frame):
    raise InterruptedError(f"Launcher received {signal.Signals(signum).name}")


def run(command, timeout):
    previous = {
        sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        return run_child(command, timeout)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def run_child(command, timeout):
    with isolated_environment() as environment:
        print(f"ROS_DOMAIN_ID={environment['ROS_DOMAIN_ID']}: {command}", flush=True)
        with subprocess.Popen(
            command, env=environment, start_new_session=True
        ) as process:
            try:
                code = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                print(
                    f"Timed out after {timeout}s; killing process group {process.pid}",
                    flush=True,
                )
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                return 124
            except BaseException:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                raise
            if code < 0:
                print(
                    f"Child terminated by {signal.Signals(-code).name} ({-code})",
                    flush=True,
                )
                return 128 - code
            print(f"Child exited with code {code}", flush=True)
            return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not args.command or args.timeout <= 0:
        parser.error("provide a command and a positive timeout")
    try:
        return run(args.command, args.timeout)
    except OSError as error:
        print(f"Unable to run test: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
