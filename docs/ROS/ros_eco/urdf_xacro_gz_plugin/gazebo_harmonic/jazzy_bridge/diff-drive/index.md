---
tags:
    - ros
    - gazebo
    - bridge
    - jazzy
    - harmonic
    - diff drive
---

# DiffDrive


## Gazebo

```xml title="gazebo diff drive plugin"
--8<-- "docs/Simulation/Gazebo/plugins/diff_drive.xml"
```

---

## ROS

### bridge file

```yaml title="bridge.yaml"
# gz topic published by DiffDrive plugin
- ros_topic_name: "odom"
  gz_topic_name: "odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# gz topic published by DiffDrive plugin
- ros_topic_name: "tf"
  gz_topic_name: "tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

# gz topic subscribed to by DiffDrive plugin
- ros_topic_name: "cmd_vel"
  gz_topic_name: "cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```