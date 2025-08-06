---
title: Ardupilot ekf3
tags:
    - ardupilot
    - ekf3
---
{{ page_folder_links() }}

An Extended Kalman Filter (EKF) algorithm is used to estimate vehicle position, velocity and angular orientation based on rate gyroscopes, accelerometer, compass, GPS, airspeed and barometric pressure measurements.


| Parameter  | value  | Desc  |
|---|---|---|
| EK3_ENABLE  | 1  |   |
| AHRS_EKF_TYPE  | 3  |   |
| EK3_IMU_MASK  |   |   |
| EK3_PRIMARY  |   |   |
| EK3_ALT_M_NSE  |   | Lower number reduces reliance on accelerometers, increases reliance on barometer.  |
| EK3_GPS_TYPE  |   | Controls how GPS is used.  |
| EK3_YAW_M_NSE  |   | Controls the weighting between GPS and Compass when calculating the heading  |


The EKF instantiates multiple instances of the filter called ‘lanes’. The primary lane is the one that provides state estimates, rest are updated in the background and available for switching to. The number of possible lanes is exactly equal to the number of IMUs enabled for use. Affinity is a way for the EKF lanes to use non-primary sensors within any running lane.

### EK3_IMU_MASK and EKF3 instance

[EK3_IMU_MASK](https://ardupilot.org/copter/docs/parameters.html#ek3-imu-mask-bitmask-of-active-imus) 1 byte bitmap of IMUs to use in EKF3. A separate instance of EKF3 will be started for each IMU selected.


| Bit | IMU | Binary | Meaning   |
| --- | --- | ------ | --------- |
| 0   | 1   | 1      | Use IMU 1 |
| 1   | 2   | 2      | Use IMU 2 |
| 2   | 3   | 4      | Use IMU 3 |


!!! note "Version 4.6 EKF deprecated parameters"
     

#### Configuration

```
EK3_IMU_MASK = 2
EK3_PRIMARY = 0   # Because only one EKF instance will run, and it's at index 0
```

```
EK3_IMU_MASK = 3 # IMU1 and IMU2 working and create two EKF instance
EK3_PRIMARY = 1   # To use the instance assigned to IMU2
```

## Sensor affinity
ArduPilot EKF3 allows you to assign specific sensors (e.g., IMUs, GPS, barometers) to particular EKF "lanes" (instances). Each EKF lane can run in parallel, using different sensor combinations for redundancy and robustness.

---

## Mavlink estimation

| Feature            | `VISION_POSITION_ESTIMATE`                | `GLOBAL_VISION_POSITION_ESTIMATE`           |
| ------------------ | ----------------------------------------- | ------------------------------------------- |
| MAVLink ID         | `102`                                     | `101`                                       |
| Frame of Reference | **Local frame** (e.g., body or map-local) | **Global frame** aligned to NED from origin |
| Position Fields    | `x, y, z` → **meters**                    | `x, y, z` → **meters**                      |
| Orientation        | `roll, pitch, yaw` in **radians**         | `roll, pitch, yaw` in **radians**           |
| Origin Definition  | Arbitrary local frame                     | Aligned to a global origin (NED or ENU)     |
| Units              | All in SI (meters, radians)               | All in SI (meters, radians)                 |
| Usage              | VIO, MoCap in local environments          | Global alignment (with known origin)        |


# TODO:Parse flag from command 193

---

<div>
    <div class="grid-item">
        <a href="gps_no_gps">
                <p>Switch ekf3 sources</p>
            </a>
    </div>
    <div class="grid-item">
        <a href="gcs">
                <p></p></a>
    </div>
     <div class="grid-item">
        <a href="optical_flow">
            <p> </p>
        </a>
    </div>
</div>

---
## Reference
- [Extended Kalman Filter (EKF)](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html)
- [EKF3 Affinity and Lane Switching](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html)
- [EKF Source Selection and Switching](https://ardupilot.org/copter/docs/common-ekf-sources.html)
- [EKF2 Affinity and Lane switching](https://discuss.ardupilot.org/t/gsoc-20-ekf3-affinity-and-lane-switching-merged/61320)
- [Non-GPS Position Estimation](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
- [Extended Kalman Filter Navigation Overview and Tuning](https://ardupilot.org/dev/docs/extended-kalman-filter.html#extended-kalman-filter)
- [GPS / Non-GPS Transitions](https://ardupilot.org/plane/docs/common-non-gps-to-gps.html)