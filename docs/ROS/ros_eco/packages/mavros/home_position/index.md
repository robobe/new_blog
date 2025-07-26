---
title: Mavros home_position plugin
tags:
  - ros
  - mavros
  - home_position
---
{{ page_folder_links() }}

Contains the home position. The home position is the default position that the system will return to and land on (RTL).
Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach


Get and Set `home_position` using mavlink message [HOME_POSITION](https://mavlink.io/en/messages/common.html#HOME_POSITION) and [SET_HOME_POSITION](https://mavlink.io/en/messages/common.html#SET_HOME_POSITION)


!!! warning "SET_HOME_POSITION deprecated"
    Replace by [MAV_CMD_DO_SET_HOME (179)](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME)
    The command expose by [command plugin](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/command.cpp) 
     

## Mavros

Publish home position as `mavros_msgs::msg::HomePosition` message under topic `` ~/home
The message publish once when successful it cancel timer the send `GET_HOME_POSITION` command every 10s


!!! warning "MAV_CMD_GET_HOME_POSITION DEPRECATED"
    And replace with [MAV_CMD_REQUEST_MESSAGE(512)](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE)

     
## Ardupilot
The `home_position` set every time the vehicle ARM and can be change using `MAV_CMD_DO_SET_HOME` mavlink message

!!! warning "EKF Origin"
    The “EKF origin” is the location that the EKF (aka AHRS) uses for internal calculations. This location is normally set to the vehicle’s location soon after the GPS provides a good quality location,Once set the EKF origin cannot be moved, the EKF origin publish my `GPS_GLOBAL_ORIGIN` and handle by MAVROS `global_position` plugin.
     