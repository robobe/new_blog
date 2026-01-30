---
title: Gazebo classic yaml launch file
tags:
    - gazebo
    - classic
    - yaml
    - launch 
    - ros
---

## key used
- set_env
- arg
- include

```yaml
launch:
  - arg:
      name: "robot_description_file"
      default: "$(find-pkg-share robot_description)/urdf/diffbot.urdf"

  - arg:
      name: "world_file"
      default: "slam_training.world"
  - arg:
      name: "use_sim_time"
      default: "true"

  - set_env:
      name: GAZEBO_RESOURCE_PATH
      value: "$(env GAZEBO_RESOURCE_PATH):$(find-pkg-share robot_gazebo)/worlds"

  # Gazebo Classic
  - include:
      file: "$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"
      arg:
        - name: "world"
          value: "$(var world_file)"
        - name: "verbose"
          value: "true"

  # robot_state_publisher (still needed for TF/robot_description)
  - node:
      pkg: "robot_state_publisher"
      exec: "robot_state_publisher"
      name: "robot_state_publisher"
      output: "screen"
      param:
        - name: "use_sim_time"
          value: "$(var use_sim_time)"
        - name: "robot_description"
          value: "$(command '$(find-exec xacro) $(var robot_description_file)')"

  # Spawn robot into Gazebo Classic
  - node:
      pkg: "gazebo_ros"
      exec: "spawn_entity.py"
      output: "screen"
      args: "-topic robot_description -entity robot -x -0.5 -y 5.0"

```