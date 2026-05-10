---
title: Gazebo simulation light
tags:
    - gazebo
    - light
---

In Gazebo Harmonic / SDF, the main light types are:

- point
- spot
- directional

They differ by:

- how light is emitted
- whether there is direction
- whether intensity decays with distance
- whether the light is focused

[sdf spec](https://sdformat.org/spec/1.12/light/)

### Real world analogy

| Real object       | Gazebo type |
| ----------------- | ----------- |
| Candle            | point       |
| Flashlight        | spot        |
| Sun               | directional |
| Car headlight     | spot        |
| Ceiling bulb      | point       |
| Stadium projector | spot        |


## Demo Sun:

!!! info "light / sun gizmo"
    directional light ignore position
    the gazebo gui mark the light as green square that shown in the pose tag, it is just a gizmo
    

```xml
<light type="directional" name="sun">
    <cast_shadows>true</cast_shadows>
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.5 0.1 -0.9</direction>
</light>
```