---
title: PyBullet wind simulation
tags:
    - pybullet
    - simulation
    - wind
    - force
---

In PyBullet there is no global built-in wind field like in some world simulators.
Wind is usually simulated by calculating an aerodynamic force in Python and
applying that force to a body or to a specific link on every simulation step.

The idea is:

- Define wind velocity in world coordinates.
- Read the body or link velocity.
- Compute the relative air velocity.
- Convert that relative velocity into force.
- Apply the force with `applyExternalForce`.

## Wind as velocity

Wind is air moving relative to the world. In simulation, represent wind as a
velocity vector:

```python
wind_velocity = [5.0, 0.0, 0.0]  # m/s, wind blowing in +X
```

If an object is also moving, the force depends on the air velocity relative to
the object:

```text
v_rel = v_wind - v_link
```

Where:

- `v_wind` is the wind velocity in world frame.
- `v_link` is the link linear velocity in world frame.
- `v_rel` is the airflow seen by the link.

If the link moves with the wind at the same speed, `v_rel = 0`, so the wind does
not push it. If the link moves against the wind, `v_rel` becomes larger and the
force increases.

## Wind force model

A common simple drag / wind force model is:

```text
F = 1/2 * rho * Cd * A * |v_rel| * v_rel
```

Where:

| Symbol | Meaning |
| ------ | ------- |
| `F` | force vector applied to the link, Newton |
| `rho` | air density, about `1.225 kg/m^3` at sea level |
| `Cd` | drag coefficient, depends on shape |
| `A` | reference area facing the wind, square meter |
| `v_rel` | relative air velocity vector, m/s |
| `|v_rel|` | speed of relative air velocity |

This equation is vector form. The term `|v_rel| * v_rel` means the force grows
with velocity squared and points in the airflow direction.

Example values:

| Shape | Approximate `Cd` |
| ----- | ---------------- |
| flat plate | `1.0 - 1.3` |
| cube / box | `0.8 - 1.1` |
| sphere | `0.4 - 0.5` |
| streamlined body | `0.05 - 0.2` |

For a simple simulation, start with:

```python
AIR_DENSITY = 1.225
DRAG_COEFFICIENT = 1.0
AREA = 0.1
```

## Apply wind to a base body

For a single rigid body, use link index `-1`. In PyBullet, `-1` means the base
link.

```python
import math
import pybullet as p


def wind_force(wind_velocity, body_velocity, area, drag_coefficient, air_density=1.225):
    v_rel = [
        wind_velocity[0] - body_velocity[0],
        wind_velocity[1] - body_velocity[1],
        wind_velocity[2] - body_velocity[2],
    ]

    speed = math.sqrt(v_rel[0] ** 2 + v_rel[1] ** 2 + v_rel[2] ** 2)

    return [
        0.5 * air_density * drag_coefficient * area * speed * v_rel[0],
        0.5 * air_density * drag_coefficient * area * speed * v_rel[1],
        0.5 * air_density * drag_coefficient * area * speed * v_rel[2],
    ]


wind_velocity = [5.0, 0.0, 0.0]
area = 0.1
cd = 1.0

linear_velocity, angular_velocity = p.getBaseVelocity(bodyUniqueId=box_id)
force = wind_force(wind_velocity, linear_velocity, area, cd)
position, orientation = p.getBasePositionAndOrientation(box_id)

p.applyExternalForce(
    objectUniqueId=box_id,
    linkIndex=-1,
    forceObj=force,
    posObj=position,
    flags=p.WORLD_FRAME,
)
```

Call this before `p.stepSimulation()` on every step.

```python
while True:
    linear_velocity, _ = p.getBaseVelocity(box_id)
    force = wind_force(wind_velocity, linear_velocity, area, cd)
    position, _ = p.getBasePositionAndOrientation(box_id)

    p.applyExternalForce(box_id, -1, force, position, p.WORLD_FRAME)
    p.stepSimulation()
```

## Apply wind to a robot link

For a robot, each joint has a child link. Wind can be applied to one link by
using the link index.

```python
link_index = 2

link_state = p.getLinkState(
    bodyUniqueId=robot_id,
    linkIndex=link_index,
    computeLinkVelocity=True,
)

link_world_position = link_state[0]
link_linear_velocity = link_state[6]

force = wind_force(
    wind_velocity=[5.0, 0.0, 0.0],
    body_velocity=link_linear_velocity,
    area=0.05,
    drag_coefficient=1.1,
)

p.applyExternalForce(
    objectUniqueId=robot_id,
    linkIndex=link_index,
    forceObj=force,
    posObj=link_world_position,
    flags=p.WORLD_FRAME,
)
```

The force changes the link motion through the robot dynamics. If the link is
connected by joints, the force can also create joint torques because the wind is
pushing on one part of the kinematic chain.

## Force position and torque

Force does not only have magnitude and direction. It also has an application
point.

If the force is applied at the center of mass, it mostly creates translation.
If the force is applied away from the center of mass, it also creates torque:

```text
tau = r x F
```

Where:

- `tau` is torque.
- `r` is the vector from center of mass to force application point.
- `F` is the wind force.
- `x` is the cross product.

In PyBullet this is controlled by `posObj`:

```python
p.applyExternalForce(
    objectUniqueId=robot_id,
    linkIndex=link_index,
    forceObj=force,
    posObj=link_world_position,
    flags=p.WORLD_FRAME,
)
```

To make wind create rotation, apply the force to a point offset from the link
center:

```python
force_point = [
    link_world_position[0],
    link_world_position[1],
    link_world_position[2] + 0.2,
]

p.applyExternalForce(robot_id, link_index, force, force_point, p.WORLD_FRAME)
```

## Controlling wind during simulation

The simplest control is to change the wind velocity over time:

```python
wind_velocity = [wind_speed, 0.0, 0.0]
```

For GUI experiments, use PyBullet debug sliders:

```python
wind_x_slider = p.addUserDebugParameter("wind x", -20.0, 20.0, 0.0)
wind_y_slider = p.addUserDebugParameter("wind y", -20.0, 20.0, 0.0)
wind_z_slider = p.addUserDebugParameter("wind z", -5.0, 5.0, 0.0)

while p.isConnected():
    wind_velocity = [
        p.readUserDebugParameter(wind_x_slider),
        p.readUserDebugParameter(wind_y_slider),
        p.readUserDebugParameter(wind_z_slider),
    ]

    link_state = p.getLinkState(robot_id, link_index, computeLinkVelocity=True)
    link_position = link_state[0]
    link_velocity = link_state[6]

    force = wind_force(wind_velocity, link_velocity, area=0.05, drag_coefficient=1.1)
    p.applyExternalForce(robot_id, link_index, force, link_position, p.WORLD_FRAME)

    p.stepSimulation()
```

## Wind gust

A gust is wind that changes with time. A simple sinusoidal gust:

```text
v_wind(t) = v_base + A * sin(2*pi*f*t)
```

Where:

- `v_base` is the steady wind speed.
- `A` is gust amplitude.
- `f` is frequency in Hz.
- `t` is simulation time.

```python
import math

base_speed = 3.0
gust_amplitude = 2.0
gust_frequency = 0.5
dt = 1.0 / 240.0
step = 0

while p.isConnected():
    t = step * dt
    wind_speed = base_speed + gust_amplitude * math.sin(2.0 * math.pi * gust_frequency * t)
    wind_velocity = [wind_speed, 0.0, 0.0]

    linear_velocity, _ = p.getBaseVelocity(box_id)
    force = wind_force(wind_velocity, linear_velocity, area=0.1, drag_coefficient=1.0)
    position, _ = p.getBasePositionAndOrientation(box_id)
    p.applyExternalForce(box_id, -1, force, position, p.WORLD_FRAME)

    p.stepSimulation()
    step += 1
```

## Important notes

- Wind is not applied automatically in PyBullet.
- `applyExternalForce` must be called every simulation step.
- Use `p.WORLD_FRAME` when the wind vector is defined in world coordinates.
- Use link index `-1` for the base and `0..N-1` for robot links.
- Lighter links move more for the same wind force because `F = m * a`.
- Joint motors, joint damping, friction, and contacts can hide the wind effect.
- Large time steps can make wind forces unstable. Start with `1 / 240` seconds.
