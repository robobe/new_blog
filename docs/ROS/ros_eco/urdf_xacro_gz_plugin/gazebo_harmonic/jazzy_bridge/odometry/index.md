---
title: Gazebo Odometry Plugin
tags:
    - ros
    - gazebo
    - bridge
    - odometry
---
{{ page_folder_links() }}

Odometry Publisher which can be attached to any entity in order to periodically publish 2D or 3D odometry data in the form of gz::msgs::Odometry messages.

## Gazebo
[more](/Simulation/Gazebo/plugins/odometry)

## messages 
| gz topic   | gz message  | ros message  |
|---|---|---|
| `/model/<name>/Odometry`  | gz.msgs.Odometry  | nav_msgs/msg/Odometry  |
| `/model/<name>/odometry_with_covariance`  | gz.msgs.OdometryWithCovariance  | nav_msgs/msg/Odometry  |
| `/model/<name>/pose` | gz.msgs.Pose_V  | tf2_msgs/msg/TFMessage  |


## Bridge

```yaml title="bridge"
- ros_topic_name: "/model/my_bot/Odometry"
  gz_topic_name: "/model/my_bot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "/model/my_bot/odometry_with_covariance"
  gz_topic_name: "/model/my_bot/odometry_with_covariance"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "/model/my_bot/pose"
  gz_topic_name: "/model/my_bot/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
```


