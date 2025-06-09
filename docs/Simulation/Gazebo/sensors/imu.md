---
tags:
    - gazebo
    - harmonic
    - sensors
    - imu
---

# IMU

- Add imu plugin in to world scope
- Add Sensor to link

```xml title="imu plugin"
<plugin
    filename="gz-sim-imu-system"
    name="gz::sim::systems::Imu">
</plugin>
```

```xml title="imu sensor"
<link>
    ...
    <sensor name="imu" type="imu">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <visualize>true</visualize>
        <topic>imu</topic>
    </sensor>
</link>
```

## imu sensor
[imu sensor](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_imu)

---

<details>
    <summary>world with imu</summary>

```xml
--8<-- "docs/Simulation/Gazebo/sensors/code/imu_world.sdf"
```
</details>

