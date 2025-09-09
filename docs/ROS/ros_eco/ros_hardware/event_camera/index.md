---
title: ROS2 event camera interface and messages
tags:
    - ros
    - event camera
---


{{ page_folder_links() }}

ROS 2 support for event cameras is provided through a set of packages designed to handle their unique data stream.  
Unlike traditional cameras that output frames at a fixed rate, **event cameras output asynchronous events**, which represent per-pixel brightness changes.


[event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs)
---

## Resource
- [event_camera_py](https://index.ros.org/p/event_camera_py/#humble)