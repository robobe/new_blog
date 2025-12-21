---
title: ROS yaml launch gazebo
tags:
  - ros
  - launch
  - yaml
  - gazebo
---

## Launch gazebo harmonic 

## Gazebo

```yaml
launch:

- executable:
    cmd: gz sim -v 4 -r empty.sdf

```

---

## Demo
- launch gazebo
- spawn robot
- spawn bridge


```yaml
launch:
  - arg:
      name: "robot_description_file"
      default: "$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro"
  - arg:
      name: "bridge_config_file"
      default: "$(find-pkg-share my_robot_bringup)/config/bridge.yaml"

  #gazebo simulation
  - executable:
      cmd: gz sim -v 4 -r empty.sdf

  # robot state publisher
  - node:
      pkg: "robot_state_publisher"
      exec: "robot_state_publisher"
      name: "robot_state_publisher"
      param:
        - name: "robot_description"
          value : "$(command '$(find-exec xacro) $(var robot_description_file)')"

  #spawn robot in gazebo
  - node:
      pkg: "ros_gz_sim"
      exec: "create"
      output: screen
      args:
          "-topic robot_description -name robot"

  #ros-gz bridge
  - node:
      pkg: "ros_gz_bridge"
      exec: "parameter_bridge"
      output: screen
      args:
          "--ros-args -p config_file:=$(var bridge_config_file)"
```