---
tags:
    - ros
    - gazebo
    - bridge
    - jazzy
    - harmonic
    - joint state
    - pose
---

# Joint state and pose publisher

## Joint state
```xml
<plugin filename="gz-sim-joint-state-publisher-system"
    name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
    <joint_name>wheel_left_joint</joint_name>
    <joint_name>wheel_right_joint</joint_name>
</plugin>
```

```yaml title="bridge"
- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```