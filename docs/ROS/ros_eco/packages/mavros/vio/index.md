---
tags:
    - ros
    - mavros
    - vio
    - odometry
    - optical flow
---


{{ page_folder_links() }}



- VISION_POSITION_ESTIMATE (102)
- VISION_VELOCITY_ESTIMATE (103)
- GLOBAL_VISION_POSITION_ESTIMATE
- ODOMETRY
- OPTICAL_FLOW
- VISION_POSITION_DELTA



| Param name  | Value  | Desc  |
|---|---|---|
| [AHRS_EKF_TYPE](https://ardupilot.org/copter/docs/parameters.html#ahrs-ekf-type-use-navekf-kalman-filter-for-attitude-and-position-estimation)  | 3  |   |
| [EK3_ENABLE](https://ardupilot.org/copter/docs/parameters.html#ek3-enable-enable-ekf3)  | 1  |   |
| [EK3_SRC1_POSXY](https://ardupilot.org/copter/docs/parameters.html#ek3-src1-posxy-position-horizontal-source-primary)  | 6  | ExternalNav  |
| EK3_SRC1_POSZ  |   |   |
| EK3_SRC1_YAW  |   |   |
|   |   |   |
|   |   |   |
|   |   |   |
|   |   |   |
|   |   |   |

[VISO_TYPE](https://ardupilot.org/copter/docs/parameters.html#viso-type-visual-odometry-camera-connection-type)





---

## Optical flow
[ekf3-optical-flow-and-gps-behavior/](https://discuss.ardupilot.org/t/ekf3-optical-flow-and-gps-behavior/95996/16)

---

## Reference
- [GPS/non-GPS Navigation Transitions - Randy Mackay](https://www.youtube.com/watch?v=3LUr0sySbgM)
- [Integration of ArduPilot and VIO tracking camera (Part 4)](https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-4-non-ros-bridge-to-mavlink-in-python/44001)