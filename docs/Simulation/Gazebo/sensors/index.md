---
tags:
    - gazebo
    - harmonic
    - sensors
---

# Sensors
- [Gazebo harmonic docs](https://gazebosim.org/docs/harmonic/sensors/){:target="_blank"}
- [Gazebo sensor list](https://gazebosim.org/docs/harmonic/comparison/){:target="_blank"}

---

<div class="grid-container">
    <div class="grid-item">
        <a href="camera">
            <p>Camera</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="nav_sat">
            <p>NavSat</p>
        </a>
    </div>
</div>

## TODO:

| Plugin            | Attach To | What It Does                      | Typical Use                  |
| ----------------- | --------- | --------------------------------- | ---------------------------- |
| **Sensors**       | World     | Manages all sensors in simulation | Required for sensors to work |
| **Imu**           | Link      | Publishes IMU data                | Orientation / control        |
| **Contact**       | Link      | Detects collisions                | Foot contact / bumpers       |
| **ForceTorque**   | Joint     | Measures forces/torques           | Grippers / joints            |
| **NavSat**        | Link      | GPS-like data                     | Outdoor robots               |
| **Magnetometer**  | Link      | Magnetic field / heading          | Yaw estimation               |
| **AirPressure**   | Link      | Pressure sensor                   | Altitude estimation          |
| **LogicalCamera** | Link      | Detects objects in view           | Object detection (simple)    |
