---
title: Mavros global position plugin
tags:
  - ros
  - mavros
  - global_position
  - ekf origin

---


| Topic  | msg  | Mavlink  | Notes  |
|---|---|---|---|
| ~/raw/fix  |   | GPS_RAW_INT  | The global position, as returned by the Global Positioning System  |
| ~/raw/gps_vel  |   |   |   |
| ~/global  | NavSatFix  | GLOBAL_POSITION_INT  | The filtered global position  |
| ~/local  |  Odometry | GLOBAL_POSITION_INT  |   |
| ~/rel_alt  |   |   |   |
| ~/compass_hdg  |   |   |   |
| ~/gp_origin  |   | GPS_GLOBAL_ORIGIN  | Publishes the GPS coordinates of the vehicle local origin (0,0,0) position. Emitted  |
| ~/gp_lp_offset  |   |   |   |
| ~/raw/satellites  |   |   |   |
| ~/set_gp_origin  |   | SET_GPS_GLOBAL_ORIGIN  | Sets the GPS coordinates of the vehicle local origin (0,0,0) position (DEPRECATED: replace by MAV_CMD_SET_GLOBAL_ORIGIN)   |



## GPS_GLOBAL_ORIGIN
Check [mavlink common](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN)
```bash
  ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{\"message_id\": 49, \"message_rate\": 2 }"
  ```

```bash title=""
# it take faw seconds until GPS fix
ros2 topic echo /mavros/global_position/gp_origin
```

```
ros2 service call /mavros/cmd/command_long mavros_msgs/srv/CommandLong "{
  broadcast: false,
  command: 409,
  confirmation: 0,
  param1: 0.0,
  param2: 0.0,
  param3: 0.0,
  param4: 0.0,
  param5: 32.461073,
  param6: <longitude>,
  param7: <altitude>
}"

```