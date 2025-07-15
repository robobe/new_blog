---
title: Ardupilot ekf3
tags:
    - ardupilot
    - ekf3
---

{{ page_folder_links() }}



| Feature            | `VISION_POSITION_ESTIMATE`                | `GLOBAL_VISION_POSITION_ESTIMATE`           |
| ------------------ | ----------------------------------------- | ------------------------------------------- |
| MAVLink ID         | `102`                                     | `101`                                       |
| Frame of Reference | **Local frame** (e.g., body or map-local) | **Global frame** aligned to NED from origin |
| Position Fields    | `x, y, z` → **meters**                    | `x, y, z` → **meters**                      |
| Orientation        | `roll, pitch, yaw` in **radians**         | `roll, pitch, yaw` in **radians**           |
| Origin Definition  | Arbitrary local frame                     | Aligned to a global origin (NED or ENU)     |
| Units              | All in SI (meters, radians)               | All in SI (meters, radians)                 |
| Usage              | VIO, MoCap in local environments          | Global alignment (with known origin)        |


---

## Reference
- [Extended Kalman Filter (EKF)](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html)
- [EKF3 Affinity and Lane Switching](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html)
- [EKF Source Selection and Switching](https://ardupilot.org/copter/docs/common-ekf-sources.html)