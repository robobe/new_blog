---
title: SET_HOME
tags:
    - ros
    - mavros
    - set_home
    - mav_cmd
---

{{ page_folder_links() }}

Sets the home position to either to the current position or a specified position. The home position is the default position that the system will return to and land on. The position is set automatically by the system during the takeoff

## Mavlink
[MAV_CMD_DO_SET_HOME (179)](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME)


| Param (Label)      | Description                                                                 | Values               | Units |
|--------------------|-----------------------------------------------------------------------------|----------------------|-------|
| 1 (Use Current)    | Use current location (`MAV_BOOL_FALSE`: use specified location). Values not equal to 0 or 1 are invalid. | `MAV_BOOL`           |       |
| 2 (Roll)           | Roll angle (of surface). Range: -180..180 degrees. `NaN` or 0 means value not set. `0.01` indicates zero roll. | min: -180, max: 180  | deg   |
| 3 (Pitch)          | Pitch angle (of surface). Range: -90..90 degrees. `NaN` or 0 means value not set. `0.01` means zero pitch. | min: -90, max: 90    | deg   |
| 4 (Yaw)            | Yaw angle. `NaN` to use default heading. Range: -180..180 degrees.          | min: -180, max: 180  | deg   |
| 5 (Latitude)       | Latitude                                                                    |                      |       |
| 6 (Longitude)      | Longitude                                                                   |                      |       |
| 7 (Altitude)       | Altitude                                                                    |                      | m     |


# without GPS
```bash title="set home position"
ros2 service call /mavros/cmd/set_home mavros_msgs/srv/CommandHome "{current_gps: false, yaw: 0.0, latitude: 32.46107376716858, longitude: 34.943326818586755, altitude: 100.0}"
```

command failed

```bsh
ros2 topic pub /mavros/set_gp_origin geographic_msgs/msg/GeoPoint "{latitude: 32.0853, longitude: 34.7818, altitude: 50.0}"

```