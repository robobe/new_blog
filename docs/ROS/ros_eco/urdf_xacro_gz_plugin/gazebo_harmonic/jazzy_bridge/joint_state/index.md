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

# Joint state

## Gazebo
```xml
<plugin filename="gz-sim-joint-state-publisher-system"
    name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
    <joint_name>wheel_left_joint</joint_name>
    <joint_name>wheel_right_joint</joint_name>
</plugin>
```

| Parameter    | Description      |
| ------------ | ------------------------------------------------------------------------------------------- |
| topic    | Name of the topic to publish to. This parameter is optional; if not provided, the joint state will be published to `/world/<world_name>/model/<model_name>/joint_state` |
| joint_name | Name of a joint to publish. This parameter can be specified multiple times and is optional. If joint names are not specified, all joints in a model will 



## messages 

| gz topic   | gz message  | ros message  |
|---|---|---|
| /world/<world_name>/model/<model_name>/joint_state  | gz.msgs.Model  | sensor_msgs/msg/JointState  |


## Bridge
```yaml title="bridge"
- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```