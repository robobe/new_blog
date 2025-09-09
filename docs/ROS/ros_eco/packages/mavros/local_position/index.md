---
title: Mavros local_position plugin
tags:
  - local_position
  - mavros
  - odometry
---

{{ page_folder_links() }}

## Topics

| topic  |   |
|---|---|
| pose  | publish position in `enu` coordinate system  |
| pose_cov  |   |
| velocity_local  | publish linear and angular velocity `enu` coordinate system angulate  |
| velocity_body  | publish linera and angular velocity `body` coordinate system (angular velocity fill from other source TODO: check)  |
| velocity_body_cov  |   |
| accel  | only `LOCAL_POSITION_NED_COV` include acceleration data, publish in `enu` coordinate system  |
| odom  |   |



## Mavlink

!!! note "mavlink site definition"
    The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)

    todo: check ardupilot message sources
     

```bash
# LOCAL_POSITION_NED
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{\"message_id\": 32, \"message_rate\": 2 }"

# LOCAL_POSITION_NED_COV
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{\"message_id\": 65, \"message_rate\": 2 }"
```


| mavlink                     | mavros topic(message) |
| --------------------------- | --------------------- |
| LOCAL_POSITION_NED (32)     |   odom                    |
| LOCAL_POSITION_NED (32)     |   pose                    |
| LOCAL_POSITION_NED (32)     |   velocity_local                    |
| LOCAL_POSITION_NED (32)     |   velocity_body                    |
| LOCAL_POSITION_NED_COV (64) |   odom                    |
| LOCAL_POSITION_NED_COV (64) |   pose_cov                    |
| LOCAL_POSITION_NED_COV (64) |   velocity_body_cov                    |
| LOCAL_POSITION_NED_COV (64) |   accel                    |

!!! tip ""
    if we register to both message
    the odom message data publish from `LOCAL_POSITION_NED_COV`
    the `tf` if enable publish from `LOCAL_POSITION_NED` data


!!!
    LOCAL_POSITION_NED_COV include acceleration data
---

## Parameters

| Name              | Desc                                       | default   |
| ----------------- | ------------------------------------------ | --------- |
| frame_id          |                                            | map       |
| tf.send           |                                            | false     |
| tf.frame_id       |                                            | map       |
| tf_child_frame_id | control to odom message **chile_frame_id** | base_link |

## nav_msgs/Odometry

The odometry carries both pose and velocity

Frames

- **header.frame_id**: The reference frame for pose (commonly odom or map)
- **child_frame_id**: The moving frame being described (commonly base_link or base_footprint)

- **Pose**: expressed in the header.frame_id (usually odom frame).
- **Velocity** : expressed in the child_frame_id (usually base_link).
        - twist.twist.linear.x → forward speed along the robot’s x-axis
        - twist.twist.linear.y → sideways (lateral) speed along the robot’s y-axis
        - twist.twist.linear.z → vertical speed (up/down)
        - twist.twist.angular → angular velocity around each axis, also in the child frame

```title="nav_msgs/Odometry"
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

---

### Mavros methods 

| Method  | Description / usage  |
|---|---|
| ftf::transform_frame_baselink_enu  |   |
| ftf::transform_frame_enu_baselink  |   |
