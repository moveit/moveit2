# Distro Compatibility Policy

This document is the contract for how moveit2's `main` branch supports multiple ROS 2 distributions simultaneously. It covers three topics:

1. **Supporting multiple distros on `main`** — patterns for in-source divergence when upstream APIs differ between distros.
2. **Handling dependencies without a bloom package** — what to do when a release exists in rosdistro but the binary deb hasn't been built yet.
3. **CI matrix policy** — the lifecycle for adding, graduating, and dropping distros from `.github/workflows/ci.yaml`.

If you're touching code that crosses a distro boundary in any of these senses, follow the pattern for that case and add a `MIGRATION.md` entry (§1.5).

---

## Topic 1: Supporting multiple distros on `main`

moveit2's `main` branch must build and pass CI on every distro listed in `.github/workflows/ci.yaml`. When upstream APIs, packages, or files diverge across those distros, divergence is handled in-source via the four patterns below — not by maintaining per-distro branches.

Every conditional block added under §1.1–§1.4 carries a **cleanup comment** naming the oldest distro that pins it (e.g. "Remove when Iron goes EOL"). When that distro goes EOL, the cleanup PR (§3) deletes the block.

### §1.1 — C++ source divergence

When a header path, type, or API differs between distros, condition the divergent code on the rclcpp version macro. rclcpp's version is the most reliable proxy for distro features because every distro ships exactly one rclcpp release.

```cpp
#include <rclcpp/version.h>

// For Rolling, Kilted, and newer
#if RCLCPP_VERSION_GTE(29, 6, 0)
#include <tf2_ros/buffer.hpp>
// For Jazzy and older
#else
#include <tf2_ros/buffer.h>
#endif
```

Version pins per distro (from Nav2):

| Distro | rclcpp version |
|---|---|
| Humble | falls into the `else` baseline |
| Jazzy | ≥ 29.0 |
| Kilted | ≥ 29.6 |
| Rolling, L-turtle | ≥ 30 |

**Reference**: [#3567](https://github.com/moveit/moveit2/pull/3567).

### §1.2 — `package.xml` dependency divergence

When upstream changes a package name (or releases a replacement under a new name) for a specific distro, use the format-3 `condition=` attribute to select the right `<*_depend>` per distro.

```xml
<!-- Remove when Iron goes EOL -->
<exec_depend condition="$ROS_DISTRO == 'humble' or $ROS_DISTRO == 'iron'">position_controllers</exec_depend>
<exec_depend condition="$ROS_DISTRO != 'humble' and $ROS_DISTRO != 'iron'">parallel_gripper_controller</exec_depend>
```

**Rules**:

- Always pair an "old" and a "new" entry with mirror conditions so every distro gets exactly one. Two entries are easier to reason about than one with multi-clause `or`.
- List the OLD entry first with an inline `<!-- Remove when … -->` comment naming the EOL trigger.

**Reference**: [REP 149](https://www.ros.org/reps/rep-0149.html#dependency).

### §1.3 — Launch-loaded files

When the *content* of a yaml/config file has to differ between distros (e.g. a controller class string that exists only on Jazzy+), use the filesystem-override convention:

> `<pkg_share>/<path>/foo.yaml` is the current/default form. A file at `<pkg_share>/<path>/foo.<distro>.yaml` overrides it for that distro only.

The launch file picks the override via filesystem feature detection — no hard-coded distro names in launch logic.

**Python launch** (`*.launch.py`):

```python
import os
from ament_index_python.packages import get_package_share_directory


def distro_specific_path(package_share: str, base_relpath: str) -> str:
    """Return a distro-overridable share-relative path.

    <pkg_share>/<base_relpath> is the default. A file at
    <pkg_share>/<stem>.<ROS_DISTRO><ext> overrides it when ROS_DISTRO is
    set and the override file exists.
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

**XML launch** (`*.launch.xml`): ROS 2 XML launch has no built-in file existence check, so use `<eval>` to embed the Python expression inline.

```xml
<launch>
  <!-- Resolve a distro-overridable yaml: config/foo.<ROS_DISTRO>.yaml if
       it exists, otherwise config/foo.yaml. -->
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

**Long-term home**: `moveit_configs_utils.MoveItConfigsBuilder` should apply this convention internally so every launch using the builder gets it for free.

**Cleanup**: when the distro that needed the override goes EOL, `rm config/foo.<distro>.yaml`. No code change required.

### §1.4 — CMake divergence

When upstream renames a package, use feature detection — try the new name first and fall back to the old. Feature detection composes cleanly with future distro additions and with downstream consumers that mix-and-match versions; it's preferred over `$ENV{ROS_DISTRO}` checks.

```cmake
# parallel_gripper_controller (Jazzy+) replaces position_controllers (Humble/Iron).
# Remove the fallback when Iron goes EOL.
find_package(parallel_gripper_controller QUIET)
if(parallel_gripper_controller_FOUND)
  set(GRIPPER_PKG parallel_gripper_controller)
else()
  find_package(position_controllers REQUIRED)
  set(GRIPPER_PKG position_controllers)
endif()

target_link_libraries(my_target PRIVATE ${GRIPPER_PKG}::${GRIPPER_PKG})
```

Use a `set(GRIPPER_PKG ...)` indirection variable so the rest of the file refers to the dep abstractly.

`$ENV{ROS_DISTRO}` conditioning is a fallback only — use it when feature detection genuinely doesn't work (e.g. two packages of the same name behaving differently). It's a code smell to lean on.

### §1.5 — MIGRATION.md updates

Every PR that introduces, modifies, or removes a distro-conditional code path must add a `MIGRATION.md` entry. This requirement applies to changes in §1.1–§1.4, Topic 2, and Topic 3 — anywhere a distro-touching divergence is added or cleaned up.

**Entry format**: date-prefixed bullet under the appropriate distro section. One sentence stating the change, one line of rationale (link the upstream PR that motivated it), and a diff block for non-obvious migrations.

```markdown
## ROS Rolling
- [06/2026] `position_controllers` was removed from `ros2_controllers` 6.7.0 ([ros-controls/ros2_controllers#2016](https://github.com/ros-controls/ros2_controllers/pull/2016)). Consumers using `position_controllers/JointGroupPositionController` should migrate to `forward_command_controller/ForwardCommandController` with `interface_name: position`. Consumers using `position_controllers/GripperActionController` should migrate to `parallel_gripper_action_controller/GripperActionController`.
```

**PR review checklist**: reviewers reject any PR that introduces a distro-conditional change without a MIGRATION.md entry.

**Cleanup**: distro release-note entries stay (they're historical). *Workaround* entries (e.g. "Remove when Resolute syncs osqp_vendor") get retired together with the workaround code.

---

## Topic 2: Handling dependencies without a bloom package

When a moveit2 dependency is **released** for a distro (entry exists in `rolling/distribution.yaml`) but **not yet bloomed** to the apt binary repo, CI fails at `rosdep install`. Two complementary approaches handle this. **Prefer §2.1.**

### §2.1 — CI source-build via `.repos` (preferred)

The CI workflow source-builds upstream deps via a per-distro `.repos` file alongside `moveit2.repos`:

```yaml
UPSTREAM_WORKSPACE: >
  moveit2.repos
  $(f="moveit2_$(sed 's/-.*$//' <<< "${{ matrix.env.IMAGE }}").repos"; test -r $f && echo $f)
```

The `sed 's/-.*$//'` strips at the first hyphen, so for `IMAGE: rolling-resolute` it looks for `moveit2_rolling.repos`. The file is currently shared across all `rolling-*` images. If image-specific files are ever needed, update the `sed` to keep the full image name.

**Rules**:

1. **Source-build only released-but-unbloomed packages.** A rosdistro entry + missing apt binary = legitimate. Source-building unreleased code is a different problem and doesn't belong here.
2. **Pin to the rosdistro `version:`** — match the package's `release:` block exactly (e.g. `version: 2.3.3-2`). Floating `main`/`master` branches drift between CI runs.
3. **Comment every entry with a removal trigger** — typically "Remove when `ros-rolling-<pkg>` deb appears for Resolute."
4. **Delete entries promptly.** Source builds are slow and re-run every CI invocation. Audit on every Rolling sync.

**Example**:

```yaml
# moveit2_rolling.repos — source builds for packages released in
# rolling/distribution.yaml but not yet bloomed to apt for Resolute.
# Remove each entry when the corresponding ros-rolling-* deb appears at
# https://repo.ros2.org/status_page/ros_rolling_default.html
repositories:
  geometric_shapes:    # rolling release 2.3.3-2, awaiting Resolute sync
    type: git
    url: https://github.com/ros-planning/geometric_shapes.git
    version: 2.3.3-2
  osqp_vendor:        # rolling release 0.2.0-4
    type: git
    url: https://github.com/tier4/osqp_vendor.git
    version: 0.2.0-4
  stomp:              # rolling release 0.1.2-4
    type: git
    url: https://github.com/ros-industrial/stomp.git
    version: 0.1.2-4
```

### §2.2 — CMake optional + feature flag (fallback)

Reach for §2.2 only when §2.1 isn't viable — e.g. the upstream is private/license-restricted, build-broken, or the CI cost of source building is prohibitive.

When a dep is genuinely missing and the consuming code can degrade gracefully, use `QUIET` + a presence check. Always pair the optional `find_package` with a `target_compile_definitions` feature flag so source code can `#ifdef`-guard usages.

```cmake
# osqp_vendor not yet bloomed for Resolute; degrade gracefully when absent.
find_package(osqp QUIET)
if(osqp_FOUND)
  add_subdirectory(online_signal_smoothing)
  target_compile_definitions(my_target PRIVATE MOVEIT_HAVE_OSQP)
endif()
```

In source:

```cpp
#ifdef MOVEIT_HAVE_OSQP
  // ... use osqp ...
#endif
```

**Don't** silently skip a subdirectory or install rule without making the absence detectable. Downstream consumers must be able to ask "is this feature available?" without reading moveit's CMake.

**Cleanup is heavier than §2.1**: removing the conditional requires deleting the CMake guard, the source `#ifdef` blocks, and the `MOVEIT_HAVE_*` macro. Audit at every Rolling sync.

---

## Topic 3: CI matrix policy

`.github/workflows/ci.yaml` is the canonical statement of "what moveit2 supports." Every distro listed there must build and pass on `main`; anything that doesn't is either an experimental job (non-blocking) or a bug to fix.

### Adding a distro

When REP 2000 declares a new distro supported, add a `<distro>-ci` entry to the matrix with the standard `IMAGE` / `ROS_DISTRO` keys. If a prebuilt `moveit/moveit2:<distro>-ci` Docker image isn't ready yet, follow the Resolute pattern from [#3743](https://github.com/moveit/moveit2/pull/3743): add an experimental entry without `DOCKER_IMAGE` (so industrial_ci builds on `ubuntu:<codename>` from `ros-testing`), set `continue-on-error: true`, and suffix the job name with `(experimental)`.

### Graduating an experimental job to required

When an experimental job is consistently green — or red only on externally-blocked items that are tracked via §2.1 — promote it by dropping `continue-on-error` and the `(experimental)` suffix. §2.1 source-build entries should be in place before graduation so the job has a real chance to pass.

### Dropping a distro

When a distro goes EOL per [REP 2000](https://www.ros.org/reps/rep-2000.html), open a single cleanup PR that:

1. Removes the matrix entry from `ci.yaml`.
2. Removes `<*_depend condition="…">` entries pinned to the dropped distro (§1.2).
3. Deletes `config/foo.<dropped-distro>.yaml` override files (§1.3).
4. Removes `find_package(... QUIET)` + indirection variables that only served the dropped distro (§1.4), or simplifies them to single-path if a newer fallback exists.
5. Adds a MIGRATION.md entry noting the support drop (§1.5).

**Where the policy lives**: this document plus the matrix file. Don't fork the support set into a separate "supported distros" list anywhere else — the matrix is the truth.

### Per-distro branches

Each non-EOL ROS distro has a corresponding long-lived branch on `moveit/moveit2` (`humble`, `jazzy`, `kilted`, ...). These branches are **stabilization-only**: bug fix backports from `main`, and nothing else. They are the basis for `catkin_prepare_release` + `bloom-release` for that distro's stream.

**Workflow file presence** — only this minimal set lives on per-distro branches; everything else is `main`-only:

| Workflow                | main | per-distro branch |
|-------------------------|:---:|:---:|
| `ci.yaml`               | ✓   | ✓ |
| `docker.yaml`           | ✓   | ✓ |
| `docker_lint.yaml`      | ✓   | ✓ |
| `format.yaml`           | ✓   | ✓ |
| `prerelease.yaml`       | ✓   | — |
| `sonar.yaml`            | ✓   | — |
| `stale.yaml`            | ✓   | — |
| `tutorial_docker.yaml`  | ✓   | — |

Rationale: per-distro branches don't have the breadth of churn `main` does, so the tutorial/prerelease/sonar/stale machinery is wasted overhead. Anything found by those workflows on a per-distro branch is also found on `main` first.

**`ci.yaml` policy on per-distro branches**:
- Matrix: exactly two entries — `<distro>-ci` with `CCOV: true`, and `<distro>-ci` with `IKFAST_TEST: true` + `CLANG_TIDY: pedantic`. No deprecation-check, no resolute-experimental, no extra noise — that's all `main`-only.
- Triggers: `workflow_dispatch`, `pull_request`, `merge_group`, `push: branches: [<distro>]`. Note the branch name in the `push` trigger must match the actual branch — a common fork-from-main bug is leaving it as `[main]`.

**`docker.yaml` policy on per-distro branches**:
- `ROS_DISTRO: [<distro>]` in every job's matrix.
- `push: branches: [<distro>]` so pushes to the branch automatically rebuild the per-distro `moveit/moveit2:<distro>-{ci,release,source}` images.

### Forking a new per-distro branch

When a new ROS distro is declared stable (e.g., L-turtle), fork the per-distro branch from `main` and apply these distro-specific edits in a single bootstrap commit:

1. **`ci.yaml`**:
   - Replace the multi-entry matrix with the two-entry per-distro shape above.
   - Change `push: branches: [main]` → `[<distro>]`.
2. **`docker.yaml`**:
   - Change every `ROS_DISTRO: [rolling]` matrix → `[<distro>]` (typically 3 occurrences across the release/ci/source jobs).
   - Change `push: branches: [main]` → `[<distro>]`.
3. **Delete** the workflows that don't live on per-distro branches: `prerelease.yaml`, `sonar.yaml`, `stale.yaml`, `tutorial_docker.yaml`.
4. **Trigger `docker.yaml` once manually** via `workflow_dispatch` on the new branch (or push any commit to it) so the buildfarm produces the initial `moveit/moveit2:<distro>-ci` Docker image. Subsequent CI runs on PRs against the branch then have a real image to pull.

**Reference**: [#3769](https://github.com/moveit/moveit2/pull/3769) for the kilted-branch retrofit that established this pattern.
