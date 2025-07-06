---
title: Gazebo differential driver
tags:
    - gazebo
    - plugins
    - diff drive
---


{{ page_folder_links() }}

Differential drive controller which can be attached to a model with any number of left and right wheels. 
[more](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html#details)

### simple gazebo declaration
```xml
<plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diff-drive-system">

    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_radius>0.1</wheel_radius>


    <max_linear_acceleration>0.33</max_linear_acceleration>

    <topic>cmd_vel</topic>


    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_topic>odom</odom_topic>
    <odom_publisher_frequency>30</odom_publisher_frequency>

    <tf_topic>/tf</tf_topic>

  </plugin>
```

## Config
| Parameter                     | Description |
|------------------------------|-------------|
| left_joint                   | Name of a joint that controls a left wheel. This element can appear multiple times, and must appear at least once. |
| right_joint                  | Name of a joint that controls a right wheel. This element can appear multiple times, and must appear at least once. |
| wheel_separation             | Distance between wheels, in meters. Optional, default is 1.0 m. |
| wheel_radius                 | Wheel radius in meters. Optional, default is 0.2 m. |
| odom_publish_frequency       | Odometry publication frequency. Optional, default is 50 Hz. |
| topic                        | Custom topic this system subscribes to for command velocity messages. Optional, default is `/model/{name_of_model}/cmd_vel`. |
| odom_topic                   | Custom topic for publishing odometry messages. Optional, default is `/model/{name_of_model}/odometry`. |
| tf_topic                     | Custom topic for publishing transform from `frame_id` to `child_frame_id`. Optional, default is `/model/{name_of_model}/tf`. |
| frame_id                     | Frame used as the origin in the odometry transform (`tf_topic` and `odom_topic`). Optional, default is `{name_of_model}/odom`. |
| child_frame_id               | Frame used as the target in the odometry transform. Optional, default is `{name_of_model}/{name_of_link}`. |
| min_velocity                 | Minimum linear and angular velocity. |
| max_velocity                 | Maximum linear and angular velocity. |
| min_acceleration             | Minimum linear and angular acceleration. |
| max_acceleration             | Maximum linear and angular acceleration. |
| min_jerk                     | Minimum linear and angular jerk. |
| max_jerk                     | Maximum linear and angular jerk. |
| min_linear_velocity          | Minimum linear velocity. Overrides `min_velocity` if set. |
| max_linear_velocity          | Maximum linear velocity. Overrides `max_velocity` if set. |
| min_angular_velocity         | Minimum angular velocity. Overrides `min_velocity` if set. |
| max_angular_velocity         | Maximum angular velocity. Overrides `max_velocity` if set. |
| min_linear_acceleration      | Minimum linear acceleration. Overrides `min_acceleration` if set. |
| max_linear_acceleration      | Maximum linear acceleration. Overrides `max_acceleration` if set. |
| min_angular_acceleration     | Minimum angular acceleration. Overrides `min_acceleration` if set. |
| max_angular_acceleration     | Maximum angular acceleration. Overrides `max_acceleration` if set. |
| min_linear_jerk              | Minimum linear jerk. Overrides `min_jerk` if set. |
| max_linear_jerk              | Maximum linear jerk. Overrides `max_jerk` if set. |
| min_angular_jerk             | Minimum angular jerk. Overrides `min_jerk` if set. |
| max_angular_jerk             | Maximum angular jerk. Overrides `max_jerk` if set. |


## usage

[moving robot tutorial](https://github.com/gazebosim/docs/blob/master/harmonic/moving_robot.md)

```bash title="gz topic publish"
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"
```