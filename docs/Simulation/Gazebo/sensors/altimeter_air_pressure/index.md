---
title: Altimeter sensor
tags:
    - gazebo
    - harmonic
    - sensors
---

## Altimeter

```xml title="plugin"
<plugin
      filename="gz-sim-altimeter-system"
      name="gz::sim::systems::Altimeter">
</plugin>
```

- Add sensor to `link`
```xml title="sensor"
<sensor name="altimeter_sensor" type="altimeter">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
</sensor>
```


## Air Pressure

```xml  title="plugin"
<plugin
    filename="gz-sim-air-pressure-system"
    name="gz::sim::systems::AirPressure">
</plugin>
```

- Add sensor to `link`
```xml title="sensor"
<sensor name="air_pressure" type="air_pressure">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>air_pressure</topic>
    <enable_metrics>true</enable_metrics>
    <air_pressure>
    <reference_altitude>123</reference_altitude>
    <pressure>
        <noise type="gaussian">
        <mean>0.2</mean>
        <stddev>0.1</stddev>
        </noise>
    </pressure>
    </air_pressure>
</sensor>
```

---

## Demo: Using Quadcopter to simulate sensors

- Altimeter


[source](code/drone.sdf)

```bash
# Takeoff
gz topic -t /X3/gazebo/command/motor_speed \
--msgtype gz.msgs.Actuators \
-p 'velocity:[700, 700, 700, 700]
```

