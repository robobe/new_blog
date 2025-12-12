---
title: ament_environment_hooks using .dev file
tags:
    - ros
    - ament_environment_hooks
    - ament
    - hooks
    - dsv
    - GZ_SIM_RESOURCE_PATH
---

`.dsv` file use to export environment variables in your ROS 2 package is a delimiter-separated description of environment variable operations.

```
<operation>;<variable>;<value>
```

| Operation               | Meaning                               |
| ----------------------- | ------------------------------------- |
| `set`                   | Overwrite the variable                |
| `prepend`               | Add a value at the beginning          |
| `prepend-non-duplicate` | Same as prepend, BUT avoid duplicates |
| `append`                | Add at the end                        |
| `append-non-duplicate`  | Same as append, but avoid duplicates  |
| `path`                  | Special logic for PATH-like variables |


## Demo

package name: bumperbot_gazebo

!!! Note dsv.in
    Becomes `.dsv` after build

```title="hooks/<project_name>.dsv.in"
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;@CMAKE_INSTALL_PREFIX@/share/@PROJECT_NAME@/worlds
```

!!! Tip substitutions can you use inside .in files
    Any CMake variable can be substituted using:
    ```
    @VARIABLE_NAME@
    ```

    examples

    | CMake variable                    | Meaning                                                |
    | --------------------------------- | ------------------------------------------------------ |
    | `@CMAKE_INSTALL_PREFIX@`          | The installation path of the package                   |
    | `@PROJECT_NAME@`                  | Name of the current CMake project (ROS 2 package name) |
    | `@PROJECT_VERSION@`               | Version from `project()`                               |
    | `@CMAKE_CURRENT_SOURCE_DIR@`      | Source folder                                          |
    | `@CMAKE_CURRENT_BINARY_DIR@`      | Build folder                                           |
    | `@CMAKE_INSTALL_FULL_LIBDIR@`     | Install lib directory                                  |
    | `@CMAKE_INSTALL_FULL_INCLUDEDIR@` | Install include directory                              |
    | `@CMAKE_PREFIX_PATH@`             | Prefix path list                                       |
    | `@CMAKE_BINARY_DIR@`              | Workspace build root                                   |
    | `@CMAKE_SOURCE_DIR@`              | Workspace source root                                  |


- Add `ament_environment_hooks` to CMakeLists.txt
```cmake
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
```

!!! Note
    - gazebo_resource.dsv.in → configured by CMake → becomes .dsv file.

    - colcon reads the .dsv file when generating local_setup.*.

    - For EACH shell it creates code based on the .dsv instructions.

- build using colcon
- source install/setup.bash
- check 

```
echo $GZ_SIM_RESOURCE_PATH
```