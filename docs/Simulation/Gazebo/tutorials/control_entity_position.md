---
tags:
    - gazebo
    - service
    - topics
    - position
    - set_pose
    - ros_gz_interfaces
---

# Control element position using set_pose service

```
 gz topic -e -t /world/my_world/pose/info
```


```
gz service --timeout 10000 -s /world/my_world/set_pose \
--reptype gz.msgs.Boolean \
--reqtype gz.msgs.Pose \
--req 'name: "green_simple_box" position {
  x: 5
  y: 0
  z: 0.5
}'
```


```
ros2 interface show ros_gz_interfaces/srv/SetEntityPose 
```

```bash title="run bridge for service"
# gz name: /world/my_world/set_pose
# ros2 service: ros_gz_interfaces/srv/SetEntityPose
ros2 run ros_gz_bridge parameter_bridge /world/my_world/set_pose@ros_gz_interfaces/srv/SetEntityPose
```

```bash title="call service from ROS"
ros2 service call /world/my_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'green_simple_box'}, \
pose: {position: {x: 10.0, y: 0.0, z: 0.5}, 
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

```

```bash title="call service from ROS"
ros2 service call /world/my_world/set_pose ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'green_simple_box'}, \
pose: {position: {x: 5.0} 
}}"
```