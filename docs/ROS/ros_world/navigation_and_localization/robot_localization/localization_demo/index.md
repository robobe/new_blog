---
tags:
    - ros
    - robot localization
---

# Robot localization using pose and twist messages

Using turtlesim read turtle odometry and create two sensors

Both sensors add random noise and set sensor covariance

- Position: publish PoseWithCovarianceStamped message
- Velocity: publish TwistWithCovarianceStamped message

Run two robot_localization nodes:

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

<details>
    <summary>map -> odom estimator</summary>


```yaml
--8<-- "docs/ROS/ros_world/navigation_and_localization/robot_localization/localization_demo/code/ekf_map.yaml"
```
</details>


<details>
    <summary>odom -> base_link estimator</summary>


```yaml
--8<-- "docs/ROS/ros_world/navigation_and_localization/robot_localization/localization_demo/code/ekf_odom.yaml"
```
</details>

The two `robot_localization" publish TF

- map-> odom
- odom -> base_link

We use the TF to draw/simulate the fuse turtle

<details>
    <summary>fuse turtle visual</summary>

```python
--8<-- "docs/ROS/ros_world/navigation_and_localization/robot_localization/localization_demo/code/tf_visual.py"
```
</details>


### Launch demo

- terminal 1: launch localization and all sensors
- terminal 2: run teleoop
- termianl 3: run fuse turtle visual

<details>
    <summary>launch </summary>

```python
--8<-- "docs/ROS/ros_world/navigation_and_localization/robot_localization/localization_demo/code/loc_sim.launch.py"
```
</details>


```bash title="terminal 1"
```

```bash title="terminal 2"
```

```bash title="terminal 3"
```
