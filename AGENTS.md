# AGENTS.md

Instructions for AI coding assistants working in this repository.

This is the single source of instructions. `CLAUDE.md` and `GEMINI.md` contain only an `@AGENTS.md` import, because Claude Code (in older versions) and Gemini CLI do not read `AGENTS.md` on their own. `.github/copilot-instructions.md` is a symlink to this file, for the Copilot Chat and IDE code-review surfaces that read only that path. Edit this file, and do not replace the import files or the symlink with copies.

## What this repo is

MoveIt 2, the ROS 2 motion planning framework. It is a multi-package colcon repository; each top-level `moveit_*` directory is a package or a group of packages, and `moveit/` is the metapackage. User docs and tutorials are in a separate repo (moveit2_tutorials, published at moveit.picknik.ai).

## Core principles

- **Fail loudly, never fake it.** MoveIt drives real robots, so a silent failure can cause physical harm. Code either does its real work or returns an error. Never leave a function that looks finished but returns placeholder data or skips steps. If you must leave something unfinished, say so to the user before you end the session.
- **Safety checks fail closed.** If a check that decides whether a robot may move (collision, joint limits, trajectory validity) cannot run, treat that the same as the check failing.
- **Keep the scope minimal.** Make the smallest change that solves the problem, and don't refactor code you were not asked to change. When your change leaves code with no remaining callers, delete that code in the same change, together with its includes, parameters, and docs.
- **Validate before calling it done.** Build the packages you touched and run their tests. If they fail, fix them instead of handing the failure to the user.

## Build, test, lint

The repo is meant to be checked out under `src/` of a ROS 2 colcon workspace (for example `ws/src/moveit2`). Run commands from the workspace root after sourcing `/opt/ros/$ROS_DISTRO/setup.bash`.

```bash
# Source dependencies not in apt (moveit_msgs, moveit_resources). On Rolling, also import moveit2_rolling.repos.
vcs import src < src/moveit2/moveit2.repos
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Build (moveit_common forces Release when CMAKE_BUILD_TYPE is unset)
colcon build --packages-up-to moveit_ros_planning
colcon build --packages-select moveit_core --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Test a package, or a single test via a ctest regex
colcon test --packages-select moveit_core
colcon test --packages-select moveit_core --ctest-args -R test_robot_state
colcon test-result --verbose
# gtest binaries can also be run directly from build/<package>/..., e.g. with --gtest_filter=Suite.Case

# Formatting and lint (required by CI): clang-format, black, codespell, cmake-format/cmake-lint
pre-commit run --all-files
```

CI (`.github/workflows/ci.yaml`) uses industrial_ci with Docker images `moveit/moveit2:<distro>-ci` (and `lyrical-ci-testing`). It builds with `-Werror` plus `-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls`, so warnings fail CI. One job runs clang-tidy (`.clang-tidy`) on changed code only, and IKFast tests run in their own job.

## Architecture

The layers depend in one direction, from bottom to top:

- **`moveit_common`**: the `moveit_package()` CMake macro. Every package calls it right after `find_package(moveit_common)`. It sets C++17 (Lyrical and Rolling raise this to C++20; see `doc/DISTRO_COMPAT_POLICY.md` §2.6), warnings, `compile_commands.json` export, and the default build type.
- **`moveit_core`**: core libraries that use ROS only for messages, `tf2` math, `pluginlib`, and `rclcpp` logging and parameters. These are robot model and state, planning scene, collision detection (FCL, Bullet, distance field), kinematic constraints and samplers, trajectory processing (TOTG, Ruckig), and online signal smoothing. It also defines the abstract **plugin interfaces** that the rest of the repo implements: `kinematics::KinematicsBase`, `planning_interface::PlannerManager`, `planning_interface::PlanningRequestAdapter`/`PlanningResponseAdapter`, `collision_detection::CollisionDetectorAllocator`, and `moveit_controller_manager::MoveItControllerManager`. Each sub-library is a separate shared library. The `moveit_core::moveit_core` INTERFACE target links them all.
- **Plugin implementations**: these are loaded at runtime through pluginlib. Each package exports `*_plugin_description.xml` files.
  - `moveit_kinematics`: KDL, cached IK, service-based IK, and the IKFast plugin generator.
  - `moveit_planners`: OMPL, Pilz industrial motion planner, CHOMP, and STOMP.
  - `moveit_plugins`: the simple controller manager and the ros2_control interface.
  - `moveit_ros/planning`: the default request and response adapters.
- **`moveit_ros`**: ROS integration that puts the core together.
  - `planning`: `RobotModelLoader`, `PlanningSceneMonitor` (keeps a planning scene in sync from topics and TF), `PlanningPipeline` (loads planner plugins plus the ordered request/response adapter chains from parameters), `TrajectoryExecutionManager`, and `MoveItCpp`, the direct C++ API that needs no `move_group` node.
  - `move_group`: the `move_group` node. Its capabilities are plugins too (`moveit_move_group_capabilities_base`).
  - `planning_interface`: `MoveGroupInterface` and `PlanningSceneInterface`, the client APIs that talk to `move_group` over ROS.
  - Also `moveit_servo` (realtime teleoperation), `hybrid_planning`, `perception`/`occupancy_map_monitor`, `visualization` (RViz plugins), `warehouse`, `trajectory_cache`, and `benchmarks`.
- **`moveit_py`**: pybind11 bindings over `moveit_core` and `MoveItCpp`.
- **`moveit_configs_utils`**: Python `MoveItConfigsBuilder`. Launch files use it to load URDF, SRDF, kinematics, joint limits, and pipeline parameters from a `*_moveit_config` package.
- **`moveit_setup_assistant`**: the GUI and library that generate those config packages.

Parameters for many components are declared with `generate_parameter_library` YAML files next to the source. Change the YAML, not hand-written `declare_parameter` calls.

User-facing API or behavior changes need an entry in `MIGRATION.md` under the distro heading. Entries are date-prefixed bullets (`- [MM/YYYY] ...`), with a diff block when the migration is not obvious.

## Rules

Topic rules live in `.claude/rules/`. Claude Code loads each one automatically when you read a file that matches its `paths:` frontmatter. Other assistants should open the file when its trigger applies.

| Trigger | Rule file |
|---|---|
| Writing or changing C++ | `.claude/rules/cpp-style.md`: language standard, naming, headers, license block, logging, Eigen |
| Code, CMake, `package.xml`, launch, `.repos`, or CI changes that differ between ROS 2 distros | `.claude/rules/distro-compat.md`, which points to `doc/DISTRO_COMPAT_POLICY.md` |

## Testing

Write the test first. The PR template asks for tests that fail without the PR. For a bug fix, write a regression test, watch it fail against the unfixed code, and then make it pass. For new behavior, write tests for the contract before the implementation.

- **Test the contract, not the implementation.** Assert on documented inputs and outputs, not on private state, so that a refactor does not break the test.
- **Cover error paths and boundaries first.** These include invalid input, an empty trajectory or a single waypoint, values exactly at joint limits, an unknown group or link name, and an empty planning scene. Then cover the typical case.
- **Include at least one test that a naive implementation would fail.** If flipping `<` to `<=`, dropping a validity check, or returning early would leave every test green, add a test that catches it.
- **Assert specific values.** Compare joint positions, poses, and durations with explicit tolerances. A test should do more than check that nothing crashed.

Put tests in the sub-library's `test/` (or `tests/`) directory, and register them in its `CMakeLists.txt` inside `if(BUILD_TESTING)`. Unit tests use `ament_add_gtest`/`ament_add_gmock`. Prefer unit tests that need no running node. For a test robot, use `moveit::core::loadTestingRobotModel("panda")` (models come from `moveit_resources`), or `moveit::core::RobotModelBuilder` for a minimal model built in code; both are in `moveit/utils/robot_model_test_utils.hpp`. Write a ROS integration test (`ament_add_gtest_executable` plus `add_ros_test(<launch>.test.py)`) only when the behavior under test is the ROS wiring itself.

## Branches, PRs, and git

- **Target `main`.** `main` must build and pass CI on every distro in the CI matrix (currently Rolling, Lyrical, Jazzy, and Humble). Mergify comments on PRs that target other branches.
- **Backports** to the stable `humble`, `jazzy`, and `kilted` branches happen through the `backport-humble`, `backport-jazzy`, and `backport-kilted` labels, which Mergify acts on after merge. Those branches take only bug fixes that keep the API and ABI. Do not open backport PRs by hand unless Mergify's backport has conflicts.
- **PRs are squash-merged**, so the PR title becomes the commit subject on `main`. Write it as a short imperative sentence ("Fix ...", "Add ...").
- **Fill in `.github/PULL_REQUEST_TEMPLATE.md`**: describe the change and link the issue, add a `MIGRATION.md` entry for API changes, add tests, and include a screenshot for GUI changes. If the change needs a tutorial update, note that it belongs in moveit2_tutorials.
- **Do not edit `CHANGELOG.rst` files.** They are regenerated in release commits.
- **CodeRabbit** reviews PRs automatically, as configured in `.coderabbit.yaml`. Each push gets an incremental review. It skips drafts and pauses after 5 reviewed commits; in those cases, post `@coderabbitai review`, or `@coderabbitai full review` to re-review commits it has already seen. Check each finding against the code before applying it, and reply in the thread when you decline one. Do not post `@coderabbitai resolve`, because it closes every thread whether or not the finding was addressed.
- **Tag AI-written GitHub text.** When you publish text to GitHub for the user (PR descriptions, issue bodies, PR and issue comments, review summaries, and inline review comments), make `[written by AI]` the first line of the body, followed by a blank line. This lets maintainers tell human-written text from generated text. Do not add the tag to commit messages, code, or code comments.
- Run `pre-commit run --all-files` before committing.
- Before any destructive git command (`reset`, `checkout -- <file>`, `clean`, force push), confirm the exact files and revisions with the user.
