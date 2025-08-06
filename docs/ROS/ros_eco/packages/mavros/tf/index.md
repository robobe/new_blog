---
title: Mavros TF's
tags:
  - ros
  - mavros
  - tf
---

{{ page_folder_links() }}

### mavros
mavros node add 3 static_tf between ENU and NED

- map
- odom
- base_link

The tf's name control by 3 parameters:

- map_frame_id
- odom_frame_id
- base_link_frame_id

|    tf frame       | global | drift | jump |  | usage |
| --------- | ------ | ----- | ---- | ---- | ---- |
| map       | V      | X     |  V   |  resocialization    |  Global localization    |
| odom      |        | V     |  X   |  smooth    |  Local motion (EKF)    |
| base_link |        |       |      |  follow the robot    |  robot body    |


!!! note "odom"
    ROS deliberately keeps odom as a local, continuous frame that doesn’t "jump" when localization improves. This is important for controllers (e.g., velocity PID or path planners) to avoid sudden changes in robot pose.
     

!!! tip "Dead Reckoning"
    Dead reckoning is a method for estimating a robot’s current position based solely on its last known position and motion over time, without external reference (like GPS or SLAM).


### Odometry