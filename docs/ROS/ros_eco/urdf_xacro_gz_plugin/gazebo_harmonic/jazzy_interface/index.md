---
tags:
    - tag
---

# ROS GZ Interface
This package currently contains some Gazebo-specific ROS message and service data structures (.msg and .srv)

[Message and service data structures for interacting with Gazebo from ROS2](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_interfaces)

## Demo: Set Entity POSE

Gazebo expose service to set entity position and orientation `/world/<world name>/set_pose`


### Run from gazebo
```bash title="gz set_pose service call"
gz service --timeout 10000 -s /world/my_world/set_pose \
--reptype gz.msgs.Boolean \
--reqtype gz.msgs.Pose \
--req 'name: "box" position {
  x: 5
  y: 0
  z: 0.5
}'
```

### Expose the service via ROS

```bash title="run bridge"
ros2 run ros_gz_bridge parameter_bridge \
    /world/my_world/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

```bash title="query entity name"
# or check the urdf
# idea: wotk with grep
gz topic -e -t /world/my_world/pose/info 
```

```bash title="call full message"
# call service set_pose on world name: my_world
# entity name: 
ros2 service call /world/my_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'box'}, \
pose: {position: {x: 3.0, y: 3.0, z: 0.5}, 
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

```

```bash
# send part of the message other argument get defaults
ros2 service call /world/my_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'box'}, \
pose: {position: {x: 1.0 } } }"

```
