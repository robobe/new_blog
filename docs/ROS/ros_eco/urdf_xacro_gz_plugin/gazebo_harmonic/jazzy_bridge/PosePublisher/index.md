---
title: Gazebo PosePublisher Plugin
tags:
    - ros
    - gazebo
    - bridge
    - PosePublisher
---

## Plugin
Pose publisher system. Attach to an entity to publish the transform of its child entities in the form of gz::msgs::Pose messages, or a single gz::msgs::Pose_V message if "use_pose_vector_msg" is true. 

[gz sim reference](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1PosePublisher.html)
### urdf

```xml
<gazebo>
   <plugin
        filename="gz-sim-pose-publisher-system"
        name="gz::sim::systems::PosePublisher">
        <publish_model_pose>true</publish_model_pose>
        <publish_link_pose>false</publish_link_pose>
        <update_rate>10</update_rate>
    </plugin>
</gazebo>
```

### bridge

```yaml
- ros_topic_name: "/bumperbot/pose"
  gz_topic_name: "/model/bumperbot/pose"
  ros_type_name: "geometry_msgs/msg/PoseStamped"
  gz_type_name: "gz.msgs.Pose"
  direction: GZ_TO_ROS
```