---
title: IMU Sensor
tags:
    - imu
    - robotics
    - sensors
---
{{ page_folder_links() }}
<!-- # IMU Sensor -->
An IMU sensor (Inertial Measurement Unit) is a device that measures a robot or object's motion and orientation using:

| Sensor                    | Measures                             | Unit  |
| ------------------------- | ------------------------------------ | ----- |
| Accelerometer             | Linear acceleration                  | m/s²  |
| Gyroscope                 | Angular velocity (rotational speed)  | rad/s |
| Magnetometer *(optional)* | Magnetic field for heading (compass) | µT    |

- **6 DOF** IMU, combining a 3-axis accelerometer and a 3-axis gyroscope.
- **9 DOF** IMU, adding a 3-axis magnetic compass.
- **10 DOF** IMU, which adds a barometer for estimating the sensor’s altitude.




---

## To read and watch
- [complementary filter](https://www.luisllamas.es/en/measure-imu-tilt-arduino-complementary-filter/)
- [madgwick_py: A Python implementation of Madgwick's IMU and AHRS algorithm.](https://github.com/morgil/madgwick_py?tab=readme-ov-file)
- [madgwick algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [Kalman Filter for 6DOF IMU Implementation](https://www.youtube.com/watch?v=Os6V1lnUPZo)