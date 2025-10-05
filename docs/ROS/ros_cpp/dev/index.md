---
title: ROS2 C++ VSCode setup
tags:
    - ros
    - vscode
    - ide
---

{{ page_folder_links() }}

Config VScode for ROS2 cpp package 
using



---
## Configuration

The `c_cpp_properties.json` file configures IntelliSense, code completion, and error checking for C/C++ projects in VS Code.

<details>
    <summary>c_cpp_properties.json for gcc</summary>

for gcc compiler
```json
--8<-- "docs/ROS/ros_cpp/dev/code/c_cpp_properties.json"
```
</details>

- **includePath**: Directories for header file lookup. Includes ROS headers and your workspace’s src folder.
- **defines**: Preprocessor macros (empty in this config).
- **cStandard and cppStandard**: C and C++ language standards (c17 and c++17). ros jazzy build with this standard
- **intelliSenseMode**: IntelliSense engine mode (linux-gcc-x64 for Linux GCC).

## Build

### Tasks

```json title=".vscode/tasks.json"

```

## Tests

```bash
colcon test --event-handlers console_direct+

# specific package
colcon test --packages-select loc_application  --event-handlers console_direct+
```