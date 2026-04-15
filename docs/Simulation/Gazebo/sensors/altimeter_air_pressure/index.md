---
title: Altimeter and Air pressure sensor
tags:
    - gazebo
    - harmonic
    - sensors
---

## Altimeter

Altitude (height) relative to a reference (usually world origin or sea level)

The sensor return `gz.msgs.Altimeter` message



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

---

## Air Pressure

Air pressure (Pa) at the sensor location return `gz.msgs.FluidPressure` message

$$P = P_0 \cdot e^{-\frac{g \cdot M \cdot h}{R \cdot T}}$$

- $P_0$ ​: pressure at sea level (~101325 Pa)
- h: altitude
- g: gravity
- T: temperature


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

