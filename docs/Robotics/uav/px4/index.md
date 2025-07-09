---
title: PX4
tags:
    - px4
---


{{ page_folder_links() }}


## TO-Read and watch
- [mavros_controllers](https://github.com/Jaeyoung-Lim/mavros_controllers): Controllers for controlling MAVs using the mavros package in OFFBOARD mode.
- [quad_controller_SE3](https://github.com/jianhengLiu/quad_controller_SE3): quadrotor controller based on PX4/mavros and SE3 geometric control
- [px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard): This repository contains a python examples for offboard control on ROS2 with PX4


```bash
make px4_sitl gz_x500_gimbal
#
SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 /home/user/git/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

## wayland
```
     QT_QPA_PLATFORM=xcb make px4_sitl gz_x500 
```

```
| Command                  | Description                |
| ------------------------ | -------------------------- |
| `uorb top`               | List active uORB topics    |
| `uorb list`              | List all registered topics |
| `listener <topic>`       | Print current message      |
| `listener <topic> -r 10` | Print at 10 Hz             |
| `uorb status`            | Show uORB system stats     |

```

---

## Resource
- [PX4-ROS2-Gazebo Drone Simulation Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template)
- [bebop_ros](https://github.com/juliordzcer/bebop_ros)
- [Aerial vehicle gazebo](https://manuals.plus/m/c5faa3a33983c8a410d6e717cdc780e179ade9fa949bf9b1320512b1439a2004)