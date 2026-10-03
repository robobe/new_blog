---
title: SLAM from beginner to advanced
tags:
    - robotics
    - slam
    - mapping
    - localization
    - navigation
---
# SLAM (Simultaneous Localization and Mapping)

SLAM (Simultaneous Localization and Mapping) is a technique used in robotics and computer vision to create a map of an unknown environment while simultaneously keeping track of the robot's location within that environment. It is a fundamental problem in robotics, particularly for autonomous vehicles, drones, and mobile robots.
SLAM algorithms typically use sensor data (such as LiDAR, cameras, or IMU) to build a map of the environment and estimate the robot's position. The process involves two main components:

1. **Mapping — “What is around me?”** Creating a representation of the environment, which can be a 2D or 3D map, depending on the application.
2. **Localization — “Where am I?”** Determining the robot's position and orientation within the created map.

## Learning path: beginning to advanced

| Stage | Topic | Main goal |
| ----- | ----- | --------- |
| 1 | [Geometry and coordinate frames](geometry_and_coordinate_frames.md) | Start with 2D frames and translation, then 3D; learn rotation and pose. |
| 2 | Sensors and odometry | Use camera, LiDAR, wheel, and IMU measurements to estimate motion. |
| 3 | Probabilistic estimation | Handle noise and covariance; learn localization, Kalman/EKF updates, and particle filters. |
| 4 | Visual odometry | Estimate camera motion between images and understand accumulated drift. |
| 5 | Basic SLAM | Estimate poses and a map together using landmarks, scan matching, and data association; explore EKF SLAM. |
| 6 | Graph SLAM | Use loop closure and nonlinear optimization to correct the trajectory and map. |
| 7 | Visual-inertial SLAM | Combine camera and IMU measurements. |
| 8 | Practical deployment | Explore ORB-SLAM3 or RTAB-Map; integrate SLAM with navigation tools such as Nav2. |

## Key Concepts in SLAM

- **Sensor Fusion**: Combining data from multiple sensors to improve the accuracy and robustness of the SLAM system.
- **Feature Extraction**: Identifying and extracting key features from the sensor data that can be used for mapping and localization.
- **Data Association**: Matching observed features with previously mapped features to maintain consistency in the map and the robot's position.
- **Loop Closure**: Recognizing when the robot has returned to a previously visited location, which helps to correct drift in the map and improve accuracy.
- **Graph-Based SLAM**: A popular approach that represents the SLAM problem as a graph, where nodes represent poses (robot positions and orientations) and edges represent constraints (observations or measurements).
- **Particle Filters**: A probabilistic approach to SLAM that uses a set of particles to represent the robot's belief about its position and the map.
- **Kalman Filters**: A mathematical approach to estimate the state of a dynamic system, for linear models with Gaussian noise; SLAM generally needs nonlinear extensions.
- **Extended Kalman Filter (EKF)**: An extension of the Kalman filter that can handle non-linear systems, commonly used in SLAM.
- **Visual SLAM (V-SLAM)**: A specific type of SLAM that uses visual data from cameras to perform mapping and localization.
- **LiDAR SLAM**: A type of SLAM that uses LiDAR sensors to create high-resolution maps of the environment.



---

## Resources

- [SLAM course](https://github.com/ariarobotics/robotic-mapping?trk=public_post_comment-text)
- [Claus Brenner SLAM lectures](https://www.youtube.com/playlist?list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm)
- [Overview of SLAM](https://medium.com/machinevision/overview-of-slam-50b7f49903b7)
- [Implement SLAM from scratch](https://medium.com/machinevision/implement-slam-from-scratch-b1fb599f40c8)

