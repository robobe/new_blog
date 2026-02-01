---
tags:
    - gazebo
    - classic
    - sensors
---

# Gazebo Classic Sensors and ROS2 plugin

<div class="grid-container">
    <div class="grid-item">
        <a href="imu">
            <p>IMU</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="-">
        </a>
    </div>
    <div class="grid-item">
        <a href="-">
        </a>
    </div>
</div>

---

## Ros2 gazebo launch and spawn urdf

```yaml title="launch file example"
launch:
  - arg:
      name: "robot_description_file"
      default: "$(find-pkg-share robot_description)/urdf/imu.urdf"

  - arg:
      name: "world_file"
      default: "empty.world"
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
      args: "-topic robot_description -entity robot "

```