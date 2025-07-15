---
title: Mavros setpoint_raw
tags:
  - ros
  - mavros
  - setpoint_raw
---

{{ page_folder_links() }}

## Mavlink

- POSITION_TARGET_LOCAL_NED (85)
- POSITION_TARGET_GLOBAL_INT(87)
- ATTITUDE_TARGET (83)
- SET_ATTITUDE_TARGET


## MAV FRAME
[Mavlink documentation](https://mavlink.io/en/messages/common.html#MAV_FRAME)
- **GLOBAL**: Global coordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default. The following modifiers may be used with "GLOBAL":
- **INT**: Latitude/longitude (in degrees) are scaled by multiplying by 1E7.
- **LOCAL**: Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF").
  
### POSITION_TARGET_LOCAL_NED(85)
Set the vehicle’s target position (as an offset in NED from the EKF origin)

### POSITION_TARGET_GLOBAL_INT(87)
Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.

---

### SET_ATTITUDE_TARGET(82)

Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system). (SET_ATTITUDE_TARGET

| Value | Name | Description                                      |
|-------|----------|--------------------------------------------------|
| 1     | ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE   | Ignore body roll rate                             |
| 2     | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE  | Ignore body pitch rate                            |
| 4     | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE    | Ignore body yaw rate                              |
| 32    | ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET         | Use 3D body thrust setpoint instead of throttle   |
| 64    | ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE         | Ignore throttle)[https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET]


### Demo: send attitude target only with RPY and thrust

mask = 7 ignore all rate part


---

## Reference
- [Copter Commands in Guided Mode¶](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-local-ned)