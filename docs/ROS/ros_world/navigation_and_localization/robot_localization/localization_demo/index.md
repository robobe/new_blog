---
tags:
    - ros
    - robot localization
---

# Robot localization using pose and twist messages

Using turtlesim read turtle odometry and create two sensors

Botch sensors add random noise and set sensor covariance

- Position: publish PoseWithCovarianceStamped message
- Velocity: publish TwistWithCovarianceStamped message

Run two robot_localization:

- odom -> base_link (config/ekf_odom.yaml)
- map -> odom (config/ekf_map.yaml)


```bash
turtlesim_loc
├── CMakeLists.txt
├── config
│   ├── ekf_map.yaml
│   └── ekf_odom.yaml
├── launch
│   └── loc_sim.launch.py
├── package.xml
├── tmux
│   └── sim.yaml
└── turtlesim_loc
    ├── __init__.py
    ├── pose_sim.py
    ├── sensors
    │   ├── __init__.py
    │   ├── position.py
    │   └── twist.py
    └── transforms
        ├── __init__.py
        └── tf_visual.py
```

## config

```yaml
--8<-- "docs/ROS/ros_world/navigation_and_localization/robot_localization/localization_pose_twist/code/ekf_config.yaml"
```