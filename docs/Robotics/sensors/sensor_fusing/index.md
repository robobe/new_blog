---
title: Sensor fusion
tags:
    - sensor
    - fusion
---

Sensor fusion: Combining tow or more data source in a way that generates a better understanding of the system [Understanding sensor fusion and tracking](https://youtu.be/6qV3YjFppuc?list=PLn8PRpmsu08ryYoBpEKzoMOveSTyS-h4a&t=46)


Sensor fusion to estimate orientation
**AHRS**: Attitude and Heading Reference System

To describe orientation we need **reference frame** and specify the rotation in some way:

- Euler angel : Roll, Pitch, Yaw
- DCM: Direction Cosine Matrix
- Quaternion


### Dead reckoning
Dead reckoning is a **navigation method** where you estimate your current position by starting from a known position and integrating motion over time.

[video explain](https://youtu.be/0rlvvYgmTvI?t=763)

Dead reckoning uses this information:

- initial position
- velocity (or distance traveled)
- direction (orientation)

Then it updates the position step by step.

$$
x_{k+1} = x_k + v_x \, dt
$$
$$
y_{k+1} = y_k + v_y \, dt
$$


---

## Reference
- [Understanding sensor fusion and tracking](https://www.youtube.com/playlist?list=PLn8PRpmsu08ryYoBpEKzoMOveSTyS-h4a)