---
title: Altitude plugin
tags:
  - ros
  - mavros
  - altitude
---

{{ page_folder_links() }}

The current system altitude.

## MAvlink

[Altitude(141)](https://mavlink.io/en/messages/common.html#ALTITUDE)

|   |   |
|---|---|
| altitude_monotonic  |   |
| altitude_amsl  |   |
| altitude_local  |   |
| altitude_relative  |   |
| altitude_terrain  |   |
| bottom_clearance  |   |


- **amsl**: above mean sea level
---

## Ardupilot

![alt text](images/ardupilot_altitude.png)


!!! warning ""
    The message remove from ardupilot code
     


```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 141, message_rate: 1.0}"
response:
mavros_msgs.srv.MessageInterval_Response(success=False)

# mavros log
[mavros_node-1] [INFO] [1753786083.525031132] [mavros.sys]: FCU: No ap_message for mavlink id (141)
[mavros_node-1] [ERROR] [1753786083.525544178] [mavros.sys]: SYS: command plugin service call failed!
```
### Check alternative message to get the information
GLOBAL_POSITION_INT (message ID 33)

    Contains alt (AMSL) and relative_alt (relative to home)

VFR_HUD (message ID 74)

    Contains alt, climb

HOME_POSITION (for reference altitude)