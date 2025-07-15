---
title: Mavros setpoint_velocity
tags:
  - ros
  - mavros
  - setpoint_velocity
---

{{ page_folder_links() }}

setpoint get ROS `twist` or `twiststamped` message and send `SET_POSITION_TARGET_LOCAL_NED` mavlink message with mask that allow only velocity and yaw rate

| Value | Name                                     | Description                       | Bit | mask |
| ----- | ---------------------------------------- | --------------------------------- | --- | ------------- |
| 1     | POSITION_TARGET_TYPEMASK_X_IGNORE        | Ignore position x                 | 0   | 1             |
| 2     | POSITION_TARGET_TYPEMASK_Y_IGNORE        | Ignore position y                 | 1   | 1             |
| 4     | POSITION_TARGET_TYPEMASK_Z_IGNORE        | Ignore position z                 | 2   | 1             |
| 8     | POSITION_TARGET_TYPEMASK_VX_IGNORE       | Ignore velocity x                 | 3   |               |
| 16    | POSITION_TARGET_TYPEMASK_VY_IGNORE       | Ignore velocity y                 | 4   |               |
| 32    | POSITION_TARGET_TYPEMASK_VZ_IGNORE       | Ignore velocity z                 | 5   |               |
| 64    | POSITION_TARGET_TYPEMASK_AX_IGNORE       | Ignore acceleration x             | 6   | 1             |
| 128   | POSITION_TARGET_TYPEMASK_AY_IGNORE       | Ignore acceleration y             | 7   | 1             |
| 256   | POSITION_TARGET_TYPEMASK_AZ_IGNORE       | Ignore acceleration z             | 8   | 1             |
| 512   | POSITION_TARGET_TYPEMASK_FORCE_SET       | Use force instead of acceleration | 9   |               |
| 1024  | POSITION_TARGET_TYPEMASK_YAW_IGNORE      | Ignore yaw                        | 10  | 1             |
| 2048  | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | Ignore yaw rate                   | 11  |               |


## SET_POSITION_TARGET_LOCAL_NED
Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).


!!! note "LOCAL"
    Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF").
     

## Parameters

| Param  |   |
|---|---|
| mav_frame  | LOCAL_NED  |
