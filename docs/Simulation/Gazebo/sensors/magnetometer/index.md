---
title: Magnetometer sensor
tags:
    - gazebo
    - harmonic
    - sensors
---

A magnetometer measures the Earth’s magnetic field vector in 3D:

$$\vec{B} = [B_x,\ B_y,\ B_z]$$

- Units: Tesla (T) (usually microtesla µT)
- Output: field strength along X, Y, Z axes of the sensor


In gazebo simulation the sensor return `gz.msgs.Magnetometer` message, the sensor add to a `Link` and return it's reading in body frame


$$\text{yaw} = \operatorname{atan2}(B_y,\ B_x)$$


```xml title="plugin"
<plugin
      filename="gz-sim-magnetometer-system"
      name="gz::sim::systems::Magnetometer">
</plugin>
```

```xml title="Add sensor to link"
<sensor name="magnetometer_sensor"  type="magnetometer">
    <always_on>1</always_on>
    <update_rate>10</update_rate>
</sensor>
```

---

## Demo: Magnetometer

- Add sensor to diff robot
- Add transport python binding and calc yaw from $B_y, B_x$


<details>
<summary>World</summary>
```
--8<-- "docs/Simulation/Gazebo/sensors/magnetometer/code/moving.sdf"
```
</details>


```python title="yaw.py"
--8<-- "docs/Simulation/Gazebo/sensors/magnetometer/code/yaw.py"
```