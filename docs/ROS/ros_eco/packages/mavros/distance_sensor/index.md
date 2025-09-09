---
title: Distance sensor
tags:
    - ros
    - mavros
    - distance_sensor
---


{{ page_folder_links() }}

## Plugin

mavros_extra [distance_sensor]()

The plugin use as publisher for mavlink DISTANCE_SENSOR(132)
And can be Subscriber and send range data into the FCU


!!! note "Rangefinder"
    Ardupilot send mavlink message [Rangefinder (173)](https://mavlink.io/en/messages/ardupilotmega.html#RANGEFINDER) There is a plugin in that handle this message mavros_extra.rangefinder

    ```bash
    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 173, message_rate: 1.0}"
    ```
     
## Topic and Mavlink message

| mavlink  | mavros topic(message) |
|---|---|
| DISTANCE_SENSOR(132)  | **rangefinder** (sensor_msgs/msg/Range)  |

!!! warning "Topic name"
    The topic name set by the config param 

    `rangefinder_pub` is the topic name for sensor id 0

    ```yaml
    /**/distance_sensor:
      ros__parameters:
        config: |
        rangefinder_pub: 
            id: 0
    ```
     
---

## usage

```bash title="set message interval"
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 132, message_rate: 1.0}"
```

### Parameters


```yaml title="para" linenums="1" hl_lines="4 11"
/**/distance_sensor:
  ros__parameters:
    config: |
      rangefinder_pub: 
        id: 0
        frame_id: "lidar"
        # orientation: PITCH_270  # sended by FCU
        field_of_view: 0.0        # XXX TODO
        send_tf: false
        sensor_position: {x:  0.0, y:  0.0, z:  -0.1}
      rangefinder_sub:
        subscriber: true
        id: 1
        orientation: PITCH_270  # only that orientation are supported by APM 3.4+
```

1. **line 4**: use as publisher topic name
1. **line 11**: use as subscriber topic name


#### Config publisher
| field  | usage  |
|---|---|
| subscriber |   default(false) |
| id  | sensor id  |
| frame_id  | use by message header `frame_id` and tf message |
| field_of_view  | sensor info  |
| send_tf  | send data as tf default(false)  |



| custom_orientation  |   |



#### Config subscriber
| field  | usage  |
|---|---|
|   |   |
| orientation  |   |
| covariance  |   |
| horizontal_fov_ratio  |   |
| vertical_fov_ratio  |   |
| sensor_position  |   |
---

## Reference
- [RangeFinders Setup Overview](https://ardupilot.org/copter/docs/common-rangefinder-setup.html)