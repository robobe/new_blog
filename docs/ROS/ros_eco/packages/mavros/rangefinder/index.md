---
title: Mavros rangefinder
tags:
    - mavros
    - rangefinder
---


{{ page_folder_links() }}
## Plugin

mavros_extra [rangefinder]()

## Topic and Mavlink message

| mavlink  | mavros topic(message) |
|---|---|
| RANGEFINDER (173) | **rangefinder** (sensor_msgs/msg/Range) |

!!! warning "Ardupilot message"
    Ardupilot send mavlink message [Rangefinder (173)](https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER) There is a plugin in that handle this message mavros_extra.rangefinder

    ```bash title="set message interval"
    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 173, message_rate: 1.0}"
    ```
     

## Simulate
Config SITL to simulate RangeFinder

```ini
RNGFND1_TYPE = 100        # Type = simulated sonar
RNGFND1_MIN_CM = 20     # Min distance (cm)
RNGFND1_MAX_CM = 700    # Max distance (cm)
RNGFND1_ORIENT = 25     # Orientation: downward-facing
RNGFND1_FUNCTION = 1       # Model = linear
```

---

TODO: Extend the plugin,
- Add variance 
- read min and max from parameter
- read rotation from parameter