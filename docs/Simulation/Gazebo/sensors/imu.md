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

```xml title="sensor with noise"
<sensor name="imu" type="imu">
    <always_on>1</always_on>
    <update_rate>50</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
    <enable_metrics>true</enable_metrics>

    <imu>
        <angular_velocity>
            <x>
                <noise type="gaussian">
                    <mean>0.0</mean>
                    <stddev>2e-4</stddev>
                    <bias_mean>0.0000075</bias_mean>
                    <bias_stddev>0.0000008</bias_stddev>
                </noise>
            </x>
            <y>
                <noise type="gaussian">
                    <mean>0.0</mean>
                    <stddev>2e-4</stddev>
                    <bias_mean>0.0000075</bias_mean>
                    <bias_stddev>0.0000008</bias_stddev>
                </noise>
            </y>
            <z>
                <noise type="gaussian">
                    <mean>0.0</mean>
                    <stddev>2e-4</stddev>
                    <bias_mean>0.0000075</bias_mean>
                    <bias_stddev>0.0000008</bias_stddev>
                </noise>
            </z>
        </angular_velocity>
        <linear_acceleration>
            <x>
                <noise type="gaussian">
                    <mean>0.0</mean>
                    <stddev>1.7e-2</stddev>
                    <bias_mean>0.1</bias_mean>
                    <bias_stddev>0.001</bias_stddev>
                </noise>
            </x>
            <y>
                <noise type="gaussian">
                    <mean>0.0</mean>
                    <stddev>1.7e-2</stddev>
                    <bias_mean>0.1</bias_mean>
                    <bias_stddev>0.001</bias_stddev>
                </noise>
            </y>
            <z>
                <noise type="gaussian">
                    <mean>0.0</mean>
                    <stddev>1.7e-2</stddev>
                    <bias_mean>0.1</bias_mean>
                    <bias_stddev>0.001</bias_stddev>
                </noise>
            </z>
        </linear_acceleration>
    </imu>
</sensor>
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
