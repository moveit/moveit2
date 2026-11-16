# Distro Compatibility Policy

This document defines how the `main` branch of moveit2 supports several ROS 2 distributions at the same time. It has four topics:

1. **Per-distro branches**: which source users and maintainers use for each distro, when a branch is cut, and how the branches are kept.
2. **Supporting multiple distros on `main`**: code patterns for differences between distros.
3. **Dependencies without an apt binary**: what to do when a release exists in rosdistro but its binary package is not yet in the apt repository.
4. **CI matrix policy**: how to add and remove distros in `.github/workflows/ci.yaml`.

If a change crosses a distro boundary, follow the pattern for that case. Add a `MIGRATION.md` entry (§2.5).

---

## Topic 1: Per-distro branches

Each distro that has not reached end of life has a long-lived branch on `moveit/moveit2`. The branches today are `humble`, `jazzy`, and `kilted`. Each branch serves apt users of its distro. Choose the source by what the user needs:

- **Ease of install and API and ABI stability**: Install the apt packages for the distro. The packages come from the distro branch. Use this for production systems and for code that must not change when the API or ABI changes.
- **The newest features, which may break the API or ABI**: Use a `main` checkout. `main` can contain API or ABI changes that the apt packages of a distro will not have.

**When to cut a branch**: Cut the branch for a non-Rolling distro from `main` just before a breaking API or ABI change merges into `main`. Releases for that distro then come from the branch. `main` receives the breaking change and moves on. A distro branch receives only bug-fix backports from `main` and must not change the API or ABI of that distro.

### Forking a distro branch

When you cut a distro branch under the rule above, fork it from `main`. Make these changes in one bootstrap commit:

1. **`ci.yaml`**: Keep only the matrix entries for `<distro>`. Change `push` to `branches: [<distro>]`.
2. **`docker.yaml`**: Keep only the `release`, `ci`, and `source` jobs. Set each `ROS_DISTRO` list to `[<distro>]`. Change `push` to `branches: [<distro>]`.
3. **Delete** `prerelease.yaml`, `sonar.yaml`, and `docker.build.yaml`.
4. **Run `docker.yaml` once** with `workflow_dispatch`, or push any commit to the branch. This builds the first `moveit/moveit2:<distro>-ci` image. Later CI runs on the branch then have an image to pull.

---

## Topic 2: Supporting multiple distros on `main`

The `main` branch must build and pass CI on every distro in `.github/workflows/ci.yaml`. Handle differences in the source tree with the patterns below. Do not use per-distro branches for this.

Each conditional block under §2.1 to §2.4 needs a **cleanup comment**. The comment names the oldest distro that needs the block, for example "Remove when Humble goes EOL". The cleanup PR in Topic 4 removes the block when that distro reaches end of life.

**Upstream links**: The PR description of each PR that adds or changes a distro difference must link the upstream PR or release that causes it. Other ROS community projects use these links to find the same fix.

### §2.1 — C++ source differences

When a header, type, or API differs between distros, guard the code. Choose the guard in this order:

1. **Header change**: use `__has_include`. It needs no version threshold, and it gives the correct result on every release line.
2. **Behavior or signature change with no header to test**: use the version macro of the package that provides the feature. Do not use `RCLCPP_VERSION` as a general distro check. Find the threshold as described under "Find the first release with the change".

| Feature | Guard on |
|---|---|
| Header rename (for example `tf2_ros` or `ament_index_cpp`) | `__has_include(<header>)` |
| Behavior or signature change in `tf2_ros` | `TF2_ROS_VERSION_GTE(...)` (from `<tf2_ros/version.h>`). `tf2` itself has no version macro. |
| Behavior or signature change in `ament_index_cpp` | `AMENT_INDEX_CPP_VERSION_GTE(...)` (from `<ament_index_cpp/version.h>`) |
| Behavior or signature change in rclcpp | `RCLCPP_VERSION_GTE(...)` (from `<rclcpp/version.h>`) |
| General distro change with no specific package | `RCLCPP_VERSION_GTE(...)`, and verify the threshold (see the `RCLCPP_VERSION` fallback) |

**Example** (header change; `__has_include` requires C++17, the oldest standard on `main`, see §2.6):

```cpp
// ament_index_cpp added get_package_share_path.hpp in 1.13.2 (Rolling line),
// 1.8.4 (Jazzy), and 1.11.4 (Kilted). Remove when Humble goes EOL.
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
```

**Example** (behavior change in `tf2_ros`, which provides its own version macro):

```cpp
// tf2_ros 0.45.0 replaced the NodeT constructor of TransformListener with
// RequiredInterfaces (geometry2 #714). tf2_ros 0.45.7 (Rolling-on-Noble) has the API but
// no version header, so this guard does not build there. Remove when Humble goes EOL.
#include <tf2_ros/version.h>
#if TF2_ROS_VERSION_GTE(0, 45, 0)
  tf2_ros::TransformListener listener(*buffer,
                                      tf2_ros::TransformListener::RequiredInterfaces{
                                          node->get_node_base_interface(), node->get_node_logging_interface(),
                                          node->get_node_parameters_interface(), node->get_node_topics_interface() },
                                      false  // spin_thread
  );
#else
  tf2_ros::TransformListener listener(*buffer, node, false /* spin_thread */);
#endif
```

**Example** (behavior change in rclcpp):

```cpp
// rclcpp 30.1.1 deprecated the free function rclcpp::spin_some(node). Use
// SingleThreadedExecutor::spin_some() instead. Remove when Humble goes EOL.
// executor is a rclcpp::executors::SingleThreadedExecutor with the node added once, before the loop.
#if RCLCPP_VERSION_GTE(30, 1, 1)
  executor.spin_some();
#else
  rclcpp::spin_some(node);
#endif
```

**Find the first release with the change** (version macros only):

1. List the release tags of the carrier package. Bloom releases map to upstream tags.
2. Check each tag for the change, for example with `git cat-file -e <tag>:<path>`.
3. Check each release line separately. A backport can add the change to an older line at a lower patch version. Do not compare version numbers across lines.
4. Link the upstream PR in the PR description. Cite the tag in a code comment and in the `MIGRATION.md` entry.

**First release with the `spin_some` deprecation** (`rclcpp`, guard `RCLCPP_VERSION_GTE(30, 1, 1)`):

| Release line | Current release | Has the deprecation? | Latest release without the deprecation |
|---|---|---|---|
| Humble | 16.0.21 | no | 16.0.21 |
| Jazzy | 28.1.22 | no | 28.1.22 |
| Kilted | 29.5.11 | no | 29.5.11 |
| Lyrical | 32.0.3 | yes | N/A (32.0.0 already has it) |
| Rolling-on-Noble (frozen) | 30.1.4 | yes | 30.1.0 |
| Rolling-on-Resolute | 33.2.0 | yes | 30.1.0 |

The first upstream tag with the deprecation is 30.1.1. Tag 30.1.0 does not have it. No earlier line has a backport. The guard therefore works on every line in this table.

The Rolling-on-Resolute values come from the `ros2-testing` suite. The main `resolute` suite does not publish Rolling packages as of 2026-11-15. ros_buildfarm PR #1151 would sync Rolling to main after each successful sync to testing. Re-check when main `resolute` lists `ros-rolling-*` packages. Compare versions only within one suite.

The frozen Rolling-on-Noble image has rclcpp 30.1.4, which already has the `spin_some` deprecation. The guard `RCLCPP_VERSION_GTE(30, 1, 1)` therefore selects the new path there. Confirm that this result is intended.

**References**: [#3567](https://github.com/moveit/moveit2/pull/3567), [#3703](https://github.com/moveit/moveit2/pull/3703), and [#3705](https://github.com/moveit/moveit2/pull/3705) (the wrong-package guard example).

### §2.2 — `package.xml` dependency differences

When a package name differs by distro, use the format-3 `condition=` attribute. Select one `<*_depend>` entry for each distro.

```xml
<!-- Remove when Humble goes EOL -->
<exec_depend condition="$ROS_DISTRO == 'humble'">position_controllers</exec_depend>
<exec_depend condition="$ROS_DISTRO != 'humble'">parallel_gripper_controller</exec_depend>
```

**Rules**:

- Pair an old entry and a new entry with mirror conditions. Each distro must match exactly one entry.
- List the old entry first. Add a `<!-- Remove when … -->` comment that names the end-of-life trigger.

**Reference**: [REP 149](https://www.ros.org/reps/rep-0149.html#dependency).

### §2.3 — Launch-loaded files

When the content of a YAML or config file differs by distro, use an override file. The default file is `<pkg_share>/<path>/foo.yaml`. The file `<pkg_share>/<path>/foo.<distro>.yaml` overrides it for that distro only.

The launch file checks whether the override exists. Do not write distro names into launch logic.

**Python launch** (`*.launch.py`):

```python
import os
from ament_index_python.packages import get_package_share_directory


def distro_specific_path(package_share: str, base_relpath: str) -> str:
    """Return the override path for the current distro, or the default path.

    The override is <pkg_share>/<stem>.<ROS_DISTRO><ext>. It is used only
    when ROS_DISTRO is set and the file exists.
    """
    distro = os.environ.get("ROS_DISTRO", "")
    if distro:
        stem, ext = os.path.splitext(base_relpath)
        override = os.path.join(package_share, f"{stem}.{distro}{ext}")
        if os.path.isfile(override):
            return override
    return os.path.join(package_share, base_relpath)


# in generate_launch_description():
pkg_share = get_package_share_directory("moveit_resources_panda_moveit_config")
ros2_controllers_path = distro_specific_path(pkg_share, "config/ros2_controllers.yaml")
```

**XML launch** (`*.launch.xml`): ROS 2 XML launch cannot check whether a file exists. Use `<eval>` with a Python expression.

```xml
<launch>
  <!-- Use config/foo.<ROS_DISTRO>.yaml if it exists. Otherwise use config/foo.yaml. -->
  <let name="_share" value="$(find-pkg-share moveit_resources_panda_moveit_config)" />
  <let name="_distro" value="$(env ROS_DISTRO '')" />
  <let name="_override" value="$(var _share)/config/ros2_controllers.$(var _distro).yaml" />
  <let name="controllers_yaml"
       value="$(eval &quot;'$(var _override)' if __import__('os').path.isfile('$(var _override)') else '$(var _share)/config/ros2_controllers.yaml'&quot;)" />

  <node pkg="controller_manager" exec="ros2_control_node" output="screen">
    <param from="$(var controllers_yaml)" />
  </node>
</launch>
```

**Long-term home**: `moveit_configs_utils.MoveItConfigsBuilder` does not apply this convention yet (checked 2026-11-14). Add it there, so every launch that uses the builder receives it.

**Cleanup**: When the distro that needs the override reaches end of life, remove `config/foo.<distro>.yaml`. No code change is needed.

### §2.4 — CMake differences

When a package is renamed, use feature detection. Look for the new name first. Use the old name only when the new one is not found. Feature detection works with new distros and with downstream users that mix versions. Prefer it to `$ENV{ROS_DISTRO}` checks.

```cmake
# parallel_gripper_controller (Jazzy and later) replaces position_controllers (Humble).
# Remove the fallback when Humble goes EOL.
find_package(parallel_gripper_controller QUIET)
if(parallel_gripper_controller_FOUND)
  set(GRIPPER_PKG parallel_gripper_controller)
else()
  find_package(position_controllers REQUIRED)
  set(GRIPPER_PKG position_controllers)
endif()

target_link_libraries(my_target PRIVATE ${GRIPPER_PKG}::${GRIPPER_PKG})
```

Use a `set(GRIPPER_PKG ...)` variable so the rest of the file uses the abstract name.

Use `$ENV{ROS_DISTRO}` only when feature detection cannot work, for example when two packages with the same name behave differently.

### §2.5 — `MIGRATION.md` updates

Every PR that adds, changes, or removes a distro-conditional code path must add a `MIGRATION.md` entry. This applies to §2.1 to §2.4, Topic 3, and Topic 4.

**Entry format**: A date-prefixed bullet under the distro section. One sentence states the change. One line gives the reason and links the upstream PR. Add a diff block when the migration is not obvious.

```markdown
## ROS Rolling
- [06/2026] `position_controllers` was removed from `ros2_controllers` 6.7.0 ([ros-controls/ros2_controllers#2016](https://github.com/ros-controls/ros2_controllers/pull/2016)). Use `parallel_gripper_action_controller/GripperActionController` instead of `position_controllers/GripperActionController`.
```

**Review checklist**: Reviewers reject a distro-conditional change that has no `MIGRATION.md` entry. For §2.1 guards, reviewers also check:

- The threshold equals the release that ships the feature. Compare with the upstream release notes, not only with a local installation.
- The guard uses the macro of the package that provides the feature. Use `RCLCPP_VERSION_GTE` only when rclcpp provides the feature.

**Cleanup**: Release-note entries remain, because they are history. Remove a workaround entry together with the workaround code.

### §2.6 — C++ language standard

Humble and Jazzy build MoveIt as C++17. On Lyrical and Rolling, targets that link `rclcpp`, which is nearly all of MoveIt, build as C++20, because their `rclcpp` exports `cxx_std_20` ([ros2/ament_cmake_ros#62](https://github.com/ros2/ament_cmake_ros/pull/62)). Write C++17 that also builds without warnings as C++20, and do not guard on `__cplusplus`. Topic 4, "Removing a distro", raises the floor.

---

## Topic 3: Dependencies without an apt binary

When a moveit2 dependency is released for a distro, its entry exists in `rolling/distribution.yaml`. If its binary is not yet in the apt repository, CI fails at `rosdep install`. For example, the package may exist only in `ros-testing` until a sync. Use §3.1 first.

### §3.1 — CI source build with `.repos` files (preferred)

CI builds these upstream dependencies from source. For each image, CI selects a `.repos` file:

```yaml
UPSTREAM_WORKSPACE: >
  moveit2.repos
  $(f="moveit2_$(sed 's/-.*$//' <<< "${{ matrix.env.IMAGE }}").repos"; test -r $f && echo $f)
```

The `sed` command keeps the text before the first hyphen. For `IMAGE: rolling-ci`, CI uses `moveit2_rolling.repos`. For `IMAGE: lyrical-ci-testing`, CI looks for `moveit2_lyrical.repos`. If that file does not exist, CI uses only `moveit2.repos`.

**Rules**:

1. **Source-build only released packages that have no apt binary yet.** Do not source-build unreleased code.
2. **Pin each entry to a commit SHA or a release tag.** Do not pin a branch. A branch changes between CI runs.
3. **Add a removal comment to each entry.** For example: "Remove when the `ros-rolling-<pkg>` deb appears for Resolute."
4. **Remove entries as soon as possible.** Source builds are slow, and CI repeats them on every run.

**Example** (from the current `moveit2_rolling.repos`, shortened):

```yaml
# Rolling-only source checkouts. Pin by SHA, not by branch.
repositories:
  osqp_vendor:
    type: git
    url: https://github.com/moveit/osqp_vendor.git
    version: 1df43ba1250bf93347a23a6a1cd481efd1cd2f68  # v1.0.0 tag
  stomp:
    type: git
    url: https://github.com/ros-industrial/stomp
    version: b1a87c80f7338caae25a5c689b876da15492aa75
```

### §3.2 — Optional CMake dependency with a feature flag (fallback)

Use §3.2 only when §3.1 cannot work. For example, the upstream source is private, does not build, or takes too long to build in CI.

When the code can run without the dependency, use `QUIET` and a presence check. Pair the optional `find_package` with a `target_compile_definitions` feature flag. Source code uses the flag in `#ifdef` blocks.

```cmake
# osqp_vendor is not yet available from apt for Resolute. Build without it when absent.
find_package(osqp QUIET)
if(osqp_FOUND)
  add_subdirectory(online_signal_smoothing)
  target_compile_definitions(my_target PRIVATE MOVEIT_HAVE_OSQP)
else()
  message(STATUS "Dropping online_signal_smoothing because osqp was not found")
endif()
```

```cpp
#ifdef MOVEIT_HAVE_OSQP
  // ... use osqp ...
#endif
```

Do not skip a subdirectory or install rule without a visible sign. Users must be able to check whether a feature exists without reading the CMake files.

**Cleanup**: Removing this fallback requires removing the CMake guard, the `#ifdef` blocks, and the `MOVEIT_HAVE_*` macro. Check for this at every Rolling sync.

---

## Topic 4: CI matrix policy

`.github/workflows/ci.yaml` defines the supported distros. Every distro in that file must build and pass on `main`. A failing job is either non-blocking (experimental) or a bug to fix.

### Adding a distro

When [REP 2000](https://www.ros.org/reps/rep-2000.html) declares a distro supported:

1. Add a matrix entry in `ci.yaml`. Add the distro to the `ROS_DISTRO` lists in `docker.yaml`.
2. If `moveit/moveit2:<distro>-ci` is not yet published, use `<distro>-ci-testing`. `docker.yaml` builds it from `.docker/ci-testing/Dockerfile`. The `lyrical` entry follows this pattern.

Earlier, [#3743](https://github.com/moveit/moveit2/pull/3743) added a non-blocking Rolling-on-Resolute job with `continue-on-error`. That pattern is no longer used on `main`.

### Experimental jobs

A job may be non-blocking. Set `continue-on-error: true` and add `(experimental)` to its name. To make it required, remove both. Add any §3.1 entries first. As of 2026-11-14, `main` has no experimental job.

### Checking version guards when Rolling changes base OS

When Rolling moves to a new Ubuntu base, package versions can jump by several majors. A §2.1 guard can then select the wrong path. In the CI matrix PR for the new base:

1. Search the tree for `*_VERSION_GTE` guards.
2. For each guard, run the steps in "Find the first release with the change" for its package and release line. Then confirm that the guard selects the correct path on every image in the matrix.
3. Change any `RCLCPP_VERSION_GTE` guard whose feature belongs to another package (§2.1).

Example: The Noble to Resolute change made some `RCLCPP_VERSION_GTE(30, 0, 0)` guards wrong. They guarded `ament_index_cpp` changes (see #3703 and #3705).

### Removing a distro

When a distro reaches end of life under [REP 2000](https://www.ros.org/reps/rep-2000.html), open one cleanup PR that:

1. Removes the distro from the matrix in `ci.yaml` and from the `ROS_DISTRO` lists in `docker.yaml`.
2. Removes the `<*_depend condition="…">` entries for that distro (§2.2).
3. Deletes the `config/foo.<distro>.yaml` override files (§2.3).
4. Removes the `find_package(... QUIET)` blocks and indirection variables that served only that distro (§2.4). Simplify them if one path remains.
5. Adds a `MIGRATION.md` entry for the removed support (§2.5).
6. If the removed distro was the last C++17 distro (§2.6), raises the default in `moveit_common/cmake/moveit_package.cmake` and updates §2.6.

The matrix in `ci.yaml` is the only list of supported distros. Do not keep a second list.
