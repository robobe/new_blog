---
title: Mavros odometry plugin
tags:
    - ros
    - mavros
    - odometry
---

{{ page_folder_links() }}

Odometry plugin implement in odom.cpp locate in mavros_extra package

```yaml
/mavros/**:
  ros__parameters:
    plugin_denylist:
      # common
      - '*'

    plugin_allowlist:
      # - sys_time
      - sys_status
      - command
      - odometry
```

```bash
  ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{\"message_id\": 331, \"message_rate\": 10 }"
  ```

!!! warning "ardupilot"
    Ardupilot not implement odometry (331) message and only received this message from to companion computer
     