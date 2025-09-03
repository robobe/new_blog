---
tags:
    - ros
    - localization
---

# Robot localization

The robot_localization package is a collection of non-linear state estimators for robots moving in 3D (or 2D) space. [more](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)  


Each of the state estimators can fuse an arbitrary number of sensors (IMUs, odometers, indoor localization systems, GPS receivers…) to track the 15 dimensional:

- **position**: x, y, z
- **orientation**: roll, pitch, yaw
- **linear velocity**: vx, vy, vz
- **angular velocity**:  roll_rate, pitch_rate, yaw_rate
- **linear acceleration**: ax, ay, az


## Coordinate systems
REP 105 [Coordinate Frames for Mobile Platforms](http://www.ros.org/reps/rep-0105.html) which describe the coordinate system conventions used in ROS

(earth) -> map -> odom -> base_link

**base_link** is rigidly attached to the mobile robot’s base. The base_link frame can be attached in any arbitrary position or orientation, but REP 103 specifies the preferred orientation of the frame as X forward, Y left and Z up. (flu)

**odom** frame is a local, drifting coordinate system used to track the robot's position relative to its starting point.
- It starts at (0, 0, 0) when the robot boots up.
- It moves with the robot as it drives around.
- It is smooth and continuous, meaning it never jumps — ideal for control.
- But it can drift over time due to sensor inaccuracies (e.g., wheel slippage, IMU noise).

Odometry is the process of estimating the robot's pose (position + orientation) by integrating motion over time.  
**odom -> base_link** transform (robot's motion relative to the start)

!!! tip "Dead Reckoning"
    Dead reckoning is the process of estimating your current position based on a previous position and motion measurements (like speed and direction).

**map** frame is a fixed, global reference frame in ROS used for global localization — a known, non-drifting coordinate system representing the world.

---

## How it's work

The robot_localization state estimator nodes accept measurements from an arbitrary number of pose-related sensors:

- **nav_msgs/Odometry**: position, orientation, linear and angular velocity
- **sensor_msgs/Imu**: orientation, angular velocity and linear acceleration
- **geometry_msgs/PoseWithCovarianceStamped**: position and orientation
- **geometry_msgs/TwistWithCovarianceStamped**: linear and angular velocity

Based on these measurements, the state estimators publish the filtered position, orientation and linear and angular velocity (nav_msgs/Odometry) on the **/odometry/filtered** topic and (if enabled) the filtered acceleration on the **/accel/filtered** topics.

### TF

robot_localization publish one of the tf's
- odom -> base_link
- map -> odom

Usually we run two estimator

- **odom → base_link** transform, which gets all continuous inputs
- **map → odom** transform, which gets all (both continuous and non-continuous)

### Sensor flow 
- **continuous sensors** send data in high rate and regularly like imu and wheel odometry (>10hz)
- **none-continuous sensor** send data in low rate like gps (<10 hz) and can be expected non-regularly interval.

This very important to the filter how much trust to put on **prediction** and how aggressively to **correct** on sensor update

---

### Covariance
Covariance is the tendency for two variables to vary together, which is a way of being correlated!

!!! note "Variance"
     Variance measures how much a single variable (e.g., sensor noise in one axis) fluctuates around its mean.

Covariance quantifies uncertainty and correlation in sensor measurements. In robotics, every sensor reading is accompanied by a covariance matrix, and estimators (like EKF-SLAM, robot_localization) use that matrix to optimally combine noisy sensor data.

[more](covariance)

---
## Demo

Use turtlesim to simulate robot_localization package usage  
The idea for the demo [The Ros Robot_localization package](https://kapernikov.com/the-ros-robot_localization-package/){:target="_blank"}

[Read more](localization_demo)

---

## Reference
- [The Ros Robot_localization package](https://kapernikov.com/the-ros-robot_localization-package/){:target="_blank"}
- [Sensor Fusion in ROS good resource](https://github.com/methylDragon/ros-sensor-fusion-tutorial){:target="_blank"}