---
title: Gazebo worlds
tags:
    - gazebo
    - gz
    - harmonic
    - world
---


The <world> element defines the global environment, which includes:

- Physics engine configuration
- Global plugins
- Time stepping
- Gravity
- Magnetic field
- Global lighting
- Scene / rendering settings
- Spherical coordinates (for GPS)
- Atmosphere model
- Global contact / sensor processing systems

## Physics engine
The default engine is DART but gazebo can run `ode` and `bullet`


!!! Tip engine type ignore
    To support back capability we set the type to ignore , if we set tag that not supported by the engine gazebo ignore them and use `DART` engine

    ```xml
    <physics name="1ms" type="ignored">
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
    </physics>
    ```