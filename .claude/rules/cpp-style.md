---
paths:
  - "**/*.cpp"
  - "**/*.hpp"
  - "**/*.h"
---

# C++ Style

Formatting is mechanical: clang-format (`.clang-format`) uses the Google base style with a 120-column limit and `SortIncludes: false`, so do not reorder includes. The rules below cover what clang-format does not.

## Language level

Write C++17 that also builds without warnings as C++20 (`doc/DISTRO_COMPAT_POLICY.md` §2.6). Use `fmt` for formatting. Under C++20 with `-Werror`, these C++17 patterns fail:

- Implicit `this` capture through `[=]`. Capture by name instead, because `[=, this]` is not valid C++17.
- Non-`const` or asymmetric comparison operators, which C++20's reversed `==` candidates can make ambiguous.
- Arithmetic or comparison between two different enum types.
- Aggregate initialization of a struct that has a user-declared constructor.

To check C++20 on Jazzy, build with `--cmake-args -DCMAKE_CXX_STANDARD=20`.

## Naming

clang-tidy (`.clang-tidy`, `readability-identifier-naming`) enforces these names, and CI runs it on changed lines:

| Entity | Style | Example |
|---|---|---|
| Classes, structs, enums, unions | `CamelCase` | `RobotState` |
| Functions and methods | `camelBack` | `getJointModelGroup()` |
| Variables and parameters | `lower_case` | `joint_names` |
| Private and protected members | `lower_case_` with a trailing underscore | `robot_model_` |
| Enum values, global, static, and class constants | `UPPER_CASE` | `DEFAULT_MAX_VELOCITY` |

clang-tidy also requires braces around statements longer than two lines and a closing `}  // namespace <name>` comment on namespaces longer than ten lines. It flags unused parameters, and `modernize-avoid-bind` flags `std::bind`, so use a lambda instead.

## Headers and files

- New headers use `.hpp`, under `<subdir>/include/moveit/<lib>/`. The `.h` files beside them are deprecated shims that only include the `.hpp`. Do not add a `.h`, and include the `.hpp` form.
- Use `#pragma once`.
- Start each new `.cpp` and `.hpp` with the BSD 3-clause license block that the neighboring files use, with your own copyright holder and year.

## Logging

Get loggers from `moveit::getLogger("moveit.<area>.<component>")` (`moveit/utils/logger.hpp`), not from `rclcpp::get_logger`. Inside a `.cpp`, the usual pattern is a `getLogger()` helper in an anonymous namespace:

```cpp
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.core.ruckig_traj_smoothing");
}
}  // namespace
```

## Eigen

Do not use `auto` for the result of an Eigen expression. Eigen returns lazy expression templates, so `auto` holds a proxy instead of a value and can dangle. Spell out the type: `const Eigen::Vector3d axis = rotation.col(2);`, not `const auto axis = rotation.col(2);`.

## `moveit_core`

`moveit_core` uses ROS only for messages, `tf2` math conversions, `pluginlib`, and `rclcpp` logging and parameter reads. Do not add subscriptions, publishers, services, actions, executors, or TF listeners to it. That integration belongs in `moveit_ros`.
