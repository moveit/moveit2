"""Tests for scripts/moveit_benchmark_statistics.py, using logs in the format of BenchmarkExecutor::writeOutput."""

import importlib.util
import os
import sqlite3
import tempfile
import unittest

SCRIPT = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..",
    "scripts",
    "moveit_benchmark_statistics.py",
)
spec = importlib.util.spec_from_file_location("moveit_benchmark_statistics", SCRIPT)
stats = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stats)

import matplotlib.pyplot as plt

PROPERTIES = [
    "final_path_correct BOOLEAN",
    "final_path_length REAL",
    "solved BOOLEAN",
    "time REAL",
]


def run_line(solved, correct, length, time):
    """One run, as BenchmarkExecutor writes it: failed runs leave the path metrics empty."""
    if solved:
        values = ["true" if correct else "false", str(length), "true", str(time)]
    else:
        values = ["", "", "false", str(time)]
    return "".join(value + "; " for value in values)


def write_log(path, planner_runs):
    """Write a MoveIt benchmark log. planner_runs maps planner names to lists of run lines."""
    lines = [
        "MoveIt version 2.14.0",
        "Experiment panda_reach",
        "Running on testhost",
        "Starting at 2026-11-26T06:00:00",
        "<<<|",
        "Motion plan request:",
        "  group_name: panda_arm",
        "|>>>",
        "0 is the random seed",
        "1 seconds per run",
        "-1 MB per run",
        "4 runs per planner",
        "2.5 seconds spent to collect the data",
        "0 enum types",
        "%d planners" % len(planner_runs),
    ]
    for planner, runs in planner_runs.items():
        lines += [planner, "0 common properties"]
        lines.append("%d properties for each run" % len(PROPERTIES))
        lines += PROPERTIES
        lines.append("%d runs" % len(runs))
        lines += runs
        lines.append(".")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# RRTConnect: 4 runs, 3 solved, 2 of those correct. PRM: 2 runs, both solved and correct.
PLANNER_RUNS = {
    "RRTConnectkConfigDefault (ompl)": [
        run_line(True, True, 1.5, 0.05),
        run_line(True, False, 1.7, 0.06),
        run_line(True, True, 1.6, 0.04),
        run_line(False, False, 0.0, 1.0),
    ],
    "PRMkConfigDefault (ompl)": [
        run_line(True, True, 2.0, 0.5),
        run_line(True, True, 2.1, 0.6),
    ],
}


class TestBenchmarkStatistics(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.log = os.path.join(self.tmpdir.name, "benchmark.log")
        self.db = os.path.join(self.tmpdir.name, "benchmark.db")
        write_log(self.log, PLANNER_RUNS)
        stats.readBenchmarkLog(self.db, [self.log])
        stats.computeViews(self.db)

    def tearDown(self):
        plt.close("all")
        self.tmpdir.cleanup()

    def test_is_true(self):
        """
        Test that BOOLEAN values logged by MoveIt ("true"/"false") and OMPL (1/0) are read correctly.
        """
        for value in ["true", "True", "1", 1]:
            self.assertTrue(stats.isTrue(value), value)
        for value in ["false", "False", "0", 0]:
            self.assertFalse(stats.isTrue(value), value)

    def test_boolean_plot_percentages(self):
        """
        Test that BOOLEAN attributes are plotted as the percentage of true values among the runs that report them.
        """
        conn = sqlite3.connect(self.db)
        cur = conn.cursor()
        cur.execute("SELECT id, name FROM plannerConfigs ORDER BY id")
        planners = cur.fetchall()

        stats.plotAttribute(cur, planners, "solved", "BOOLEAN")
        heights = [bar.get_height() for bar in plt.gca().patches]
        self.assertEqual(len(heights), 2)
        self.assertAlmostEqual(heights[0], 75.0)  # 3 of 4 runs solved
        self.assertAlmostEqual(heights[1], 100.0)

        # Failed runs have no value for final_path_correct, so they are left out.
        stats.plotAttribute(cur, planners, "final_path_correct", "BOOLEAN")
        heights = [bar.get_height() for bar in plt.gca().patches]
        self.assertAlmostEqual(heights[0], 200.0 / 3.0)  # 2 of 3 solved runs correct
        self.assertAlmostEqual(heights[1], 100.0)
        conn.close()

    def test_plot_statistics_writes_pdf(self):
        """
        Test that plotStatistics writes a PDF for a MoveIt log whose planners have different run counts.
        """
        pdf = os.path.join(self.tmpdir.name, "plots.pdf")
        stats.plotStatistics(self.db, pdf)

        self.assertTrue(os.path.isfile(pdf))
        with open(pdf, "rb") as f:
            self.assertEqual(f.read(5), b"%PDF-")


if __name__ == "__main__":
    unittest.main()
