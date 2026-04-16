---
title: ROS2 Gazebo project
tags:
    - ros
    - gazebo
    - project
    - template
---

## Demo: Launch gazebo and spawn robot
- ROS2 gazebo project build using minimal 4 packages
  - robot_application: project logic
  - robot_bringup: launch and config
  - robot_description: urdf and models
  - robot_gazebo: gazebo world and plugins
- Run gazebo using `executable`
- Spawn robot using `robot_description`

```yaml
launch:
  - arg:
      name: "bridge_config_file"
      default: "$(find-pkg-share robot_bringup)/config/bridge.yaml"

  - arg:
      name: "world_name"
      default: "robot.world"


    #gazebo simulation
  - executable:
      cmd: gz sim -v 4 -r $(var world_name)
      output: screen
      env:
        - name: GZ_SIM_RESOURCE_PATH
          value: "$(env GZ_SIM_RESOURCE_PATH):$(find-pkg-share robot_gazebo)/worlds"

    #ros-gz bridge
  - node:
      pkg: "ros_gz_bridge"
      exec: "parameter_bridge"
      output: screen
      args:
          "--ros-args -p config_file:=$(var bridge_config_file)"

  - node:
      pkg: robot_state_publisher
      exec: robot_state_publisher
      name: robot_state_publisher
      output: screen
      param:
        - name: robot_description
          value: "$(command 'xacro $(find-pkg-share robot_description)/urdf/robot.xacro')"
          
  - node:
      pkg: ros_gz_sim
      exec: create
      output: screen
      args:
          "-entity my_robot -topic robot_description"
          
```

```yaml title="bridge.yaml"
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

---

## Resources
- [Guide to ros_gz_project_template for ROS 2 and Gazebo Development](https://gazebosim.org/docs/harmonic/ros_gz_project_template_guide/){:target="_blank"}
- [github: ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template/tree/main){:target="_blank"}


```
├── loc_bringup
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── gazebo.launch.py
│   └── package.xml
└── loc_gazebo
    ├── CMakeLists.txt
    ├── hooks
    │   ├── loc_gazebo.dsv.in
    │   └── loc_gazebo.sh.in
    ├── package.xml
    └── worlds
        └── world.sdf
```


### gazebo package
```title="loc_gazebo.dsv"
prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share;@CMAKE_INSTALL_PREFIX@/share/loc_gazebo/worlds
```

```title="loc_gazebo.sh"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/@PROJECT_NAME@/worlds"
```

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(loc_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY
    worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

```

---

### Bringup package

```python title="gazebo.launch.py"
--8<-- "docs/ROS/ros_eco/urdf_xacro_gz_plugin/gazebo_harmonic/project_template/code/gazebo.launch.py"
```
