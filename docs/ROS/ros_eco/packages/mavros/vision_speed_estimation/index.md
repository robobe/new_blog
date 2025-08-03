---
title: Vision Speed Estimation
tags:
  - ros
  - mavros
  - mavros_extra
  - vision_speed
---

{{ page_folder_links() }}

## Mavlink

[VISION_SPEED_ESTIMATE (103) ](https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE) Speed estimate from a vision source.


Here is your data converted into a Markdown table:

| Field Name      | Type        | Units   | Description                                                                                                                                                                                                                  |
|-----------------|-------------|---------|--------------------------------------------------------------------------|
| usec            | uint64_t    | us      | Timestamp (UNIX time or time since system boot)                                                                                                                                                                              |
| x               | float       | m/s     | Global X speed                                                                                                                                                                                                              |
| y               | float       | m/s     | Global Y speed                                                                                                                                                                                                              |
| z               | float       | m/s     | Global Z speed                                                                                                                                                                                                              |
| covariance ++   | float[9]    |         | Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.                                      |
| reset_counter++ | uint8_t     |         | Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). Used when e.g. SLAM detects a loop-closure and the estimate jumps.        |


!!! note "Explain Global"
    "global" often refers to a frame that is fixed relative to the **Earth**, as opposed to being fixed to the vehicle's **body**. However, it's not truly global in the sense of latitude and longitude (which would be a WGS84 ECEF or similar frame). Instead, **it refers to a local tangent plane approximation of the Earth.**

    **Why not use "Local X/Y/Z speed"**  
    The reason they might use "Global" instead of "Local" is to distinguish it from a "body frame" velocity. A body frame velocity would be relative to the vehicle's own orientation


!!! note "Global (WGS84) vs. Global (Local NED)"

     

---

## Mavros
**vision_speed** plugin
