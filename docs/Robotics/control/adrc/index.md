---
title: ADRC 
tags:
    - control
---

## Learning path

1. PID on mass system
2. Add unknown disturbance
3. Add observer that estimates velocity
4. Extend observer to estimate disturbance
5. Cancel disturbance in control law


## The code ADRC idea is
- Do not perfectly model the disturbance
- Estimate it
- Cancel it


---

## Mass System Simulation

Start with the simplest case: a unit mass moving along one axis with no
disturbance.

```text
a = u
```

The controller command `u` directly creates acceleration `a`. Acceleration does
not change position directly. It first changes velocity, and velocity changes
position:

```text
v_dot = a
x_dot = v
```

That is why the state of the mass needs two values:

- `x`: position, where the mass is
- `v`: velocity, how fast the position is changing

If the simulation only stored `x`, it would not know how the mass is moving
between control updates. The velocity `v` is the memory of previous acceleration.

With a small timestep `dt`, the no-disturbance simulation is:

```python
a = u
v += a * dt
x += v * dt
```

After that basic model is working, add the unknown disturbance. The environment
adds `d` to the acceleration, so the plant becomes:

```text
a = u + d
```

Because this is a unit mass, force and acceleration have the same numeric value.
For a different mass `m`, the model would be `a = (u + d) / m`.

The simulation advances the continuous system with a small fixed timestep `dt`.
At each step:

1. Read the current position as the measurement `y`.
2. Let the controller compute a new command `u`.
3. Compute the disturbance at the current time.
4. Update acceleration, velocity, and position.

The plant update is Euler integration:

```python
d = 0.5 * np.sin(2 * t)
a = u + d
v += a * dt
x += v * dt
```

In `code/simple.py`, the simulation uses `dt = 0.001` seconds for `T = 5.0`
seconds. The reference position is `1.0`, so the controller tries to move the
mass from `x = 0` to `x = 1` while rejecting the sinusoidal disturbance.

---

## ADRC in `code/simple.py`

Active Disturbance Rejection Control (ADRC) treats everything that is not the
known control input as a disturbance and estimates it online. In
[`simple.py`](code/simple.py), the plant is a simple mass:

```python
a = u + d
```

In this example, the system is a point mass moving in one dimension. The state
of the mass is its position `x` and velocity `v`. The controller sends a command
`u`, and the plant turns the total force into acceleration:

- `a`: acceleration of the mass
- `u`: control input chosen by the controller
- `d`: unknown disturbance added by the real system

So `u` is the part we control, and `d` is the part we do not control. If there
were no disturbance, the mass would accelerate only according to `u`. Because
`d` exists, the real acceleration becomes `u + d`.

The controller only measures position `y = x`. It does not measure velocity
directly, and it does not know the disturbance `d`, so it uses an Extended State
Observer (ESO) to estimate all three values:

- `z1`: estimated position
- `z2`: estimated velocity
- `z3`: estimated total disturbance

For a second-order system, position changes because of velocity and velocity
changes because of acceleration:

```text
x_dot = v
v_dot = b0 * u + disturbance
```

That is why the observer needs `z1` and `z2`. ADRC then adds one extra state,
`z3`, for the unknown part of the acceleration. This includes the real external
disturbance, modeling error, friction, wrong mass estimate, or anything else
that makes the plant behave differently from `b0 * u`.

The observer compares the estimated position with the measured position:

```python
error = self.z1 - y
```

Then it corrects `z1`, `z2`, and `z3` using the gains `beta1`, `beta2`, and
`beta3`:

```python
self.beta1 = 3 * w0
self.beta2 = 3 * w0**2
self.beta3 = w0**3
```

These gains come from choosing an observer bandwidth `w0`. A larger `w0` makes
`z1`, `z2`, and `z3` react faster to measurement error, but it also amplifies
noise and can require a smaller `dt`. In this example:

```python
observer_bandwidth=30.0
controller_bandwidth=5.0
```

The observer is intentionally faster than the controller. A common starting
rule is:

```text
observer_bandwidth = 3 to 10 * controller_bandwidth
```

Here the ratio is `30 / 5 = 6`, which is a reasonable first choice.

The controller uses `z1` and `z2` like a normal PD controller:

```python
a_desired = kp * (ref - z1) + kd * (0 - z2)
```

Then it subtracts the estimated disturbance:

```python
u = (a_desired - z3) / b0
```

So the role of the three observer states is:

- `z1` tells the controller where the mass is.
- `z2` tells the controller how fast the mass is moving.
- `z3` tells the controller how much unknown acceleration should be cancelled.


---

## Reference

[adrc controller](https://www.youtube.com/watch?v=4SE_t6-DnQ4&list=PLq9ofiBVTfA4VNMVnCCFWjLdfVxZqoWKv)
- [ Active Disturbance Rejection Control the intuitive way part 1 ](https://youtu.be/DS5VEFD-r_A)
