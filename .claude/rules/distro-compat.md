---
paths:
  - "**/*.cpp"
  - "**/*.hpp"
  - "**/CMakeLists.txt"
  - "**/*.cmake"
  - "**/package.xml"
  - "**/*.launch.py"
  - "**/*.launch.xml"
  - "moveit_configs_utils/**/*.py"
  - "moveit2*.repos"
  - ".github/workflows/**"
---

# Distro Compatibility

`main` must build and pass CI on every distro in the `.github/workflows/ci.yaml` matrix. Before you add or change code that behaves differently between ROS 2 distros, read [`doc/DISTRO_COMPAT_POLICY.md`](../../doc/DISTRO_COMPAT_POLICY.md) and use the pattern it gives for your case. It covers C++ guards, the C++17/C++20 language standard split, `package.xml` conditions, launch-loaded files, CMake, dependencies with no apt binary yet, and CI matrix changes. That document is authoritative, and this file only tells you when to read it.

Reviewers reject a distro-conditional change that is missing any of these:

- A cleanup comment that names the oldest distro needing the block, for example `// Remove when Humble goes EOL.`
- A `MIGRATION.md` entry.
- A link in the PR description to the upstream PR or release that causes the difference.

Use a per-distro branch only for bug-fix backports, never to work around a distro difference on `main`.
