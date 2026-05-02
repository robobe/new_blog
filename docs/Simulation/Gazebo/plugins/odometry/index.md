---
title: Gazebo OdometryPublisher
tags:
    - gazebo
    - plugin
    - odometry
---

{{ page_folder_links() }}

Odometry Publisher which can be attached to any entity in order to periodically publish 2D or 3D odometry data in the form of gz::msgs::Odometry messages. [more](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html#details){:target="_blank"}



```xml
<!-- add to model -->
<model>
  <plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <odom_topic>/robot/odometry</odom_topic>
    <odom_publish_frequency>50</odom_publish_frequency>
    <!-- default is 2; use 3 for full x,y,z + roll,pitch,yaw odometry -->
    <dimensions>3</dimensions>
  </plugin>
...
</model>
```

!!! note "model plugin"
     
    

### Topics

- /model/my_bot/odometry (gz.msgs.Odometry)
- /model/my_bot/odometry_with_covariance (gz.msgs.OdometryWithCovariance)
- /model/my_bot/pose (gz.msgs.Pose_V)

!!! tip "topics name"
    The default topic name

    ```xml
    /model/{model_name}/odometry
    ```


!!! info "velocity"
    Velocity data send only if the entity moving
    
---


## Config

|   |   |   |
|---|---|---|
| odom_frame  |   |   |
| xyz_offset  |   |   |
| rpy_offset  |   |   |
| gaussian_noise  | It only adds noise to the velocity (twist) part of the odometry message.  |   |
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