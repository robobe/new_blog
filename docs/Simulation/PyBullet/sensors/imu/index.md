---
title: PyBullet IMU
tags:
    - pybullet
    - simulation
    - sensors
    - imu
---

In PyBullet, an IMU is usually not a built-in sensor. You simulate it by reading the robot body state and converting it to what an IMU would measure.

Imu include two sensors:

- Accelerometer
- Gyroscope

## Accelerometer

Linear acceleration including gravity, expressed in the **IMU/body frame**

## Gyroscope
Angular velocity around IMU axes

```bash
gyro = [wx, wy, wz]  rad/s
```

## Basic implementation idea

In every simulation step 

```python
pos, orn = p.getBasePositionAndOrientation(robot_id)
lin_vel, ang_vel = p.getBaseVelocity(robot_id)
```