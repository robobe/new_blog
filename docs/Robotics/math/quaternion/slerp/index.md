---
title: SLERP
tags:
    - math
    - quaternion
    - slerp
---

# SLERP

**SLERP** means **Spherical Linear Interpolation**.

It is used to smoothly move from one orientation quaternion to another.

```text
t = 0.0  -> start orientation
t = 0.5  -> halfway orientation
t = 1.0  -> end orientation
```

SLERP is useful when you want a rotation to move smoothly and at a steady
angular speed.

Common uses:

- robot attitude interpolation
- drone or gimbal motion
- camera orientation animation
- animation between two poses

## Interactive demo

Move the `t` slider from `0` to `1`.

The muted blue body is the start orientation `q0`. The muted orange body is the
end orientation `q1`. The bright green body is the interpolated orientation
`q(t)`.

<iframe
    src="threejs/slerp_demo.html"
    width="100%"
    height="560"
    frameborder="0"
    loading="lazy">
</iframe>

## Formula

SLERP interpolates between two unit quaternions:

$$
SLERP(q_0, q_1, t)
$$

where:

- `q0` is the start orientation
- `q1` is the end orientation
- `t` is the interpolation amount from `0` to `1`

The full equation is:

$$
SLERP(q_0, q_1, t) =
\frac{\sin((1-t)\theta)}{\sin(\theta)}q_0 +
\frac{\sin(t\theta)}{\sin(\theta)}q_1
$$

`theta` is the angle between `q0` and `q1`.

!!! tip "Unit quaternions"
    SLERP expects unit quaternions. If a quaternion is not unit length,
    normalize it before interpolation.

## Why not lerp?

Linear interpolation can cut through quaternion space and may change angular
speed. SLERP follows the spherical path between orientations, so the rotation
looks smooth and natural.


---

<details>
    <summary>Demo: using scipy</summary>

```python
--8<-- "/home/user/projects/new_blog/docs/Robotics/math/quaternion/slerp/code/slerp.py"
```
</details>
