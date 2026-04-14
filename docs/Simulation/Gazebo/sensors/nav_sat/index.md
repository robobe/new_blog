---
title: GPS sensor
tags:
    - gazebo
    - harmonic
    - sensors
    - gps
---

A sensor that reports position and velocity readings over Gazebo Transport using spherical coordinates (latitude / longitude).

By default, it publishes gz::msgs::NavSat messages on the `/.../navsat` topic.

This sensor assumes the world is using the East-North-Up (**ENU**) frame. 

---

## Usage
- Add plugin (world scope)
- Add Sensor
- Set coordinate system


```xml title="plugin"
<plugin
    filename="gz-sim-navsat-system"
    name="gz::sim::systems::NavSat">
</plugin>
```

```xml title="set init position"
<spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>-22.9</latitude_deg>
    <longitude_deg>-43.2</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
</spherical_coordinates>
```

```xml title="Add sensor to link"
<sensor name="navsat_sensor" type="navsat">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
</sensor>
```

---

## Demo: Add gps to moving robot

[source](code/moving.sdf)

```json title="navsat message"
header {
  stamp {
    sec: 144
    nsec: 375000000
  }
  data {
    key: "seq"
    value: "458"
  }
}
latitude_deg: 32.085300000006519
longitude_deg: 34.781805296318218
altitude: 0.400005079805851
velocity_east: 5.1652987103355663e-13
velocity_north: 2.4066905466697168e-13
velocity_up: -2.0528406453625662e-13
frame_id: "vehicle_blue::chassis::navsat_sensor"

```

---

## Reference
- [gazebo source - demo world](https://github.com/gazebosim/gz-sim/blob/main/test/worlds/navsat.sdf)
- [gazebo docs - navsat api](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1NavSatSensor.html#details)