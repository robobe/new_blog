---
title: DO_SET_SERVO
tags:
    - ros
    - mavros
    - servo
    - mav_cmd
---

{{ page_folder_links() }}

Using mavros command to send DO_SET_SERVO command

## Mavlink
[MAV_CMD_DO_SET_SERVO (183) ](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO) Set a servo to a desired PWM value.

!!! warning "Servo function"
     SERVOx_FUNCTION must be (0) disabled for manual control

## Mavros

- Param1: Servo instance number (zero base)
- Param2: PWM value (us)


```bash
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{
  broadcast: false,
  command: 183,
  confirmation: 0,
  param1: 12,
  param2: 1750,
  param3: 0.0,
  param4: 0.0,
  param5: 0.0,
  param6: 0.0,
  param7: 0.0
}"

```