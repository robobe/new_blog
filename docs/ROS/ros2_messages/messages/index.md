---
tags:
    - ros2
    - message
    - stamped
    - covariance
    - PoseWithCovariance
---


# ROS2 Messages

## Message with covariance

### Demo: PoseWithCovariance

geometry_msgs/msg/PoseWithCovariance

```bash
ros2 interface show geometry_msgs/msg/PoseWithCovariance
```

- Pose
  - position (x,y,z)
  - orientation (q)
- covariance (float64[36])


**Covariance Matrix** : A 6*6 matrix representing uncertainties and correlations for the 6 degree of freedom (DoF)
    - x,y,z
    - roll, pith, yaw



```python
pose_covariance = [
    0.0025, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
    0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,  # y
    0.0, 0.0, 1e3, 0.0, 0.0, 0.0,   # z
    0.0, 0.0, 0.0, 1e3, 0.0, 0.0,   # roll
    0.0, 0.0, 0.0, 0.0, 1e3, 0.0,   # pitch
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001  # yaw
]
```