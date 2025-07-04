---
title: Gazebo differential driver
tags:
    - gazebo
    - plugins
    - diff drive
---


{{ page_folder_links() }}

[source code](https://github.com/gazebosim/gz-sim/tree/gz-sim9/src/systems/diff_drive)

### simple gazebo declaration
```xml
<plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diff-drive-system">

    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
    <wheel_separation>${wheel_separation}</wheel_separation>
    <wheel_radius>0.033</wheel_radius>


    <max_linear_acceleration>0.33</max_linear_acceleration>

    <topic>cmd_vel</topic>


    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_topic>odom</odom_topic>
    <odom_publisher_frequency>30</odom_publisher_frequency>

    <tf_topic>/tf</tf_topic>

  </plugin>
```

[moving robot tutorial](https://github.com/gazebosim/docs/blob/master/harmonic/moving_robot.md)

```bash title="gz topic publish"
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"
```