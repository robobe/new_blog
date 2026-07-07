---
title: Gazebo wind effect plugin
tags:
    - gazebo
    - wind
---

Gazebo can simulate wind by defining a world wind velocity and loading the
`WindEffects` system plugin. The world-level `<wind>` element sets the base wind
vector in meters per second:

```xml
<wind>
  <linear_velocity>2 0 0</linear_velocity>
</wind>
```

This value describes the direction and speed of the air flow in the world frame.
For example, `2 0 0` pushes along `+X`, while `0 1 0` pushes along `+Y`.

The `gz::sim::systems::WindEffects` plugin converts that wind field into forces
on simulated links. The plugin can also shape the wind over time. In the simple
case, `time_for_rise` controls how quickly the wind reaches the requested
magnitude:

```xml
<plugin
  filename="gz-sim-wind-effects-system"
  name="gz::sim::systems::WindEffects">
  <force_approximation_scaling_factor>1</force_approximation_scaling_factor>
  <horizontal>
    <magnitude>
      <time_for_rise>1.0</time_for_rise>
    </magnitude>
  </horizontal>
</plugin>
```

To simulate gusts, add a time-varying term such as `<sin>`. The
`amplitude_percent` changes how strong the oscillation is relative to the base
wind, and `period` controls how many seconds one cycle takes.

Wind response is applied at the link level. Enable wind on the model and on each
link that should be affected:

```xml
<model name="box">
  <enable_wind>true</enable_wind>
  <link name="link">
    <enable_wind>true</enable_wind>
  </link>
</model>
```

Whether the object visibly moves depends on the rest of its physics properties.
A lighter link accelerates more, lower ground friction lets it slide more easily,
and the collision geometry determines the approximate area exposed to the wind.
In the demos below, the box has low mass and low friction so the wind force is
easy to see.

## Demo

<details>
<summary>Wind</summary>
```
--8<-- "docs/Simulation/Gazebo/plugins/wind/code/world.sdf"
```
</details>

---

## Demo: Simulate wind gust

<details>
<summary>Wind Gust</summary>
```
--8<-- "docs/Simulation/Gazebo/plugins/wind/code/gust.sdf"
```
</details>
