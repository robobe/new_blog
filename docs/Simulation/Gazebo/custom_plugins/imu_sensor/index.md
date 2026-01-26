---
title: Gazebo plugin read sensor data
tags:
    - gazebo
    - harmonic
    - sensor
    - imu
---

## World

```xml title="add plugin in world level"
<plugin
    filename="gz-sim-imu-system"
    name="gz::sim::systems::Imu">
</plugin>
```

## Link

```xml title="add sensor at link level"
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

---

## Reference
- [gazebo tutorial sensor sdf](https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/sensors/sensor_tutorial.sdf)