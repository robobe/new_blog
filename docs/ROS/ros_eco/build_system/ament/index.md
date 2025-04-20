---
tags:
    - ros
    - build
    - ament
    - hooks
    - dsv
---

# ament

## ament_environment_hooks

Using to manage the environment variables and more when we source the workspace.
[Read more](https://github.com/gazebosim/ros_gz_project_template/blob/main/ros_gz_example_gazebo/hooks/README.md) and [more](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html)


The setup includes:

- `hook` folder
- `dsv.in` file
- `sh.in` file
- using `ament_environment_hooks` in the `CMakeLists.txt` file


### .dsv
dsv (Developer setup variables) file it use to set up the environment variables for the package.

When we source the workspace, the dsv file will be executed and set up the environment variables.

### Demo
Setup gazebo environment variables

- GZ_SIM_RESOURCE_PATH

#### Steps
- Add the `hooks` folder to your package
- Add the `dsv.in` file to your package
- Add the `sh.in` file to your package
- Add the `ament_environment_hooks` to your `CMakeLists.txt` file
- Build, Source and check

```cmake title="CMakeLists.txt"
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")
```

```bash title="ros_gz_example_description.dsv.in"
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share;@CMAKE_INSTALL_PREFIX@/share
```

```bash title="ros_gz_example_description.sh.in"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/@PROJECT_NAME@/models"
```

##### check
- Build
- source install folder

```bash
echo $GZ_SIM_RESOURCE_PATH
```

---

- [ament_cmake_extension](ament_cmake_extension.md)