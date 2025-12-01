---
title: Gazebo ROS Clock Bridge
tags:
    - ros
    - gazebo
    - bridge
    - jazzy
    - harmonic
    - clock
---

### cli

```bash
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
#
 Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
```

### bridge file

```yaml title="bridge.yaml"
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

### launch

```yaml
launch:

  - arg:
      name: "bridge_config_file"
      default: "$(find-pkg-share bumperbot_bringup)/config/bridge.yaml"

  - node:
      pkg: "ros_gz_bridge"
      exec: "parameter_bridge"
      output: screen
      args:
          "--ros-args -p config_file:=$(var bridge_config_file)"
  
```