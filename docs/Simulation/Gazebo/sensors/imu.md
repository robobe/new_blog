---
title: Gazebo IMU Sensor
tags:
  - gazebo
  - harmonic
  - sensors
  - imu
---

{{ page_folder_links() }}

The Gazebo IMU sensor simulates an Inertial Measurement Unit (IMU), providing virtual measurements of:

- Angular velocity rad/s (gyroscope)
- Linear acceleration m/s<sup>2</sup> (accelerometer)
- Orientation (quaternion)

## Coordinate system

Gazebo use ENU and body (FLU) coordinate (REP-103)

| Field               | Frame        | Note              |
| ------------------- | ------------ | ----------------- |
| Orientation         | ENU (Global) | Quaternion        |
| angular velocity    | FLU (Body)   | rad/s             |
| linear acceleration | FLU (Body)   | rad/s<sup>2</sup> |

!!! note "FLU"
    Express in body frame

    | Axis  | Direction  |
    |---|---|
    | X  | Forward  |
    | Y  | Left  |
    | Z  | Up  |


---

## message

[imu protobuf message struct ](https://github.com/gazebosim/gz-msgs/blob/gz-msgs11/proto/gz/msgs/imu.proto)

## usage

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
