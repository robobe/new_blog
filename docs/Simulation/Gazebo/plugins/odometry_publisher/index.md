---
title: Gazebo OdometryPublisher
tags:
    - gazebo
    - plugin
    - odometry
---

{{ page_folder_links() }}

Odometry Publisher which can be attached to any entity in order to periodically publish 2D or 3D odometry data in the form of gz::msgs::Odometry messages. [more](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details)


```xml
<!-- add to model -->
<plugin
    filename="gz-sim-odometry-publisher-system"
    name="gz::sim::systems::OdometryPublisher">
    <odom_frame>vehicle/odom</odom_frame>
    <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

!!! note "model plugin"
     

Add 3 topics

- /model/my_bot/odometry (gz.msgs.Odometry)
- /model/my_bot/odometry_with_covariance (gz.msgs.OdometryWithCovariance)
- /model/my_bot/pose (gz.msgs.Pose_V)


## Config

|   |   |   |
|---|---|---|
| odom_frame  |   |   |
| xyz_offset  |   |   |
| rpy_offset  |   |   |
| gaussian_noise  |   |   |
| robot_base_frame  |   |   |
| odom_publish_frequency  |   |   |
| odom_topic  |   |   |
| odom_covariance_topic  |   |   |
| tf_topic  |   |   |


## Add noise and covariance

The plugin attaches a covariance matrix to pose.covariance and twist.covariance using stddev² (noise variance).



```xml
<plugin
  filename="gz-sim-odometry-publisher-system"
  name="gz::sim::systems::OdometryPublisher">
  <!-- Use 3D odometry -->
  <dimensions>3</dimensions>

  <!-- Publishing frequency -->
  <odom_publish_frequency>50</odom_publish_frequency>

  <!-- Optional custom topics -->
  <!--
  <odom_topic>/my_robot/odom</odom_topic>
  <odom_covariance_topic>/my_robot/odom_with_covariance</odom_covariance_topic>
  -->

  <!-- Add Gaussian noise: standard deviation in meters/rad -->
  <gaussian_noise>0.05</gaussian_noise>
</plugin>
```


```bash
gz topic --echo -t /model/my_bot/odometry_with_covariance

# just the covariance part 6*6 matrix only the diagonal file with σ²
covariance {
    data: 0.0025
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0.0025
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0.0025
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0.0025
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0.0025
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0
    data: 0.0025
  }
```

## Usage
- Publish Ground truth