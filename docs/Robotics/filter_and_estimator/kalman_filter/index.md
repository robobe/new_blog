---
title: Kalman filter
tags:
    - kalman
    - filter
    - control
---


## Demo

Simple system 1D constant velocity system

### 1. State and measurement vector
$$x =
\begin{bmatrix}
p \\
v
\end{bmatrix}$$

p = position
v = velocity


### measure position

$$z = p_{measured}$$


### 2. F Matrix

The F matrix predicts the next state from the current state. To build it, write
one motion equation for each state variable at the next time step, then collect
the coefficients of the current state variables in the same order as the state
vector. Each equation becomes one row in F.

from motion equation to state transform matrix

$$p_{k+1} = p_k + v_k \Delta t$$

$$v_{k+1} = v_k$$

Notice that every equation is a linear combination of state variables.

$$p_{k+1} = 1\cdot p_k + \Delta t \cdot v_k$$

$$v_{k+1} = 0\cdot p_k + 1\cdot v_k$$

Extract coefficients

$$F=
\begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix}$$


$$F x_k
=
\begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix}
\begin{bmatrix}
p_k\\
v_k
\end{bmatrix}
=
\begin{bmatrix}
p_k & \Delta tv_k\\
& v_k
\end{bmatrix}
$$


### 3. H Matrix

The H matrix maps the internal state vector to the measurement space. In this
example the state contains position and velocity, but the sensor measures only
position, so H selects the position component and ignores velocity.

$$z = p$$

The state is: $$x =
\begin{bmatrix}
p \\
v
\end{bmatrix}$$

$$z = Hx$$

$$H =
\begin{bmatrix}
1 & 0
\end{bmatrix}$$


$$z =
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
p \\
v
\end{bmatrix}$$


### P Matrix

P is the uncertainty of your state estimate.

Because state is [position, velocity], P is 2x2

$$P =
\begin{bmatrix}
\sigma_p^2 & 0 \\
0 & \sigma_v^2
\end{bmatrix}$$

The values on the diagonal are the variance of each state variable:

- $\sigma_p^2$ is the uncertainty in the initial position estimate.
- $\sigma_v^2$ is the uncertainty in the initial velocity estimate.

The off-diagonal values describe how much the errors in position and velocity are related. If you do not know that relationship at the beginning, set them to `0`.

You often initialize `P` as an identity matrix scaled by a number:

$$P = I \cdot c =
\begin{bmatrix}
c & 0 \\
0 & c
\end{bmatrix}$$

The value `c` is your starting uncertainty. Use a large value when the initial state is only a rough guess, and a small value when the initial state comes from a reliable measurement.

For example:

$$P =
\begin{bmatrix}
100 & 0 \\
0 & 100
\end{bmatrix}$$

means the filter starts with high uncertainty in both position and velocity.


Large values mean:
“I do not trust my initial position/velocity estimate.”

Small values mean:
“I trust my initial estimate.”


### Q Matrix

Q is the process noise covariance matrix.

It describes how much uncertainty is added during the prediction step because
the motion model is not perfect. In this example, the model assumes constant
velocity:

$$p_{k+1} = p_k + v_k \Delta t$$

$$v_{k+1} = v_k$$

Real objects may accelerate, slow down, slip, or be affected by forces that are
not included in the model. Q tells the filter how much error to expect from
those unmodeled effects.

For a 1D constant-velocity model, a common Q matrix is built from acceleration
noise:

$$Q =
\sigma_a^2
\begin{bmatrix}
\frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\
\frac{\Delta t^3}{2} & \Delta t^2
\end{bmatrix}$$

where $\sigma_a^2$ is the variance of the unknown acceleration.

Large values in Q mean:
"I do not trust the motion model very much."

Small values in Q mean:
"I expect the motion model to be accurate."

If Q is too small, the filter may react slowly to real changes in motion. If Q
is too large, the estimate may become noisy because the filter expects too much
random motion.

### R Matrix

R is measurement noise.

Larger R means the filter trusts measurements less.


### Kalman filter steps
#### Predict

predict next state

$$\hat{x}_{k|k-1} = F \hat{x}_{k-1}$$

This step uses only the previous estimate and the motion model. No new sensor
measurement is used yet. The notation means:

- $\hat{x}_{k-1}$ is the best state estimate from the previous step.
- $\hat{x}_{k|k-1}$ is the predicted state at time `k`, before reading the
  measurement at time `k`.
- `F` moves the state forward by one time step according to the chosen motion
  model.

predict uncertainty

$$P_{k|k-1} = F P_{k-1} F^T + Q$$

#### Correct / Update

Measurement residual:

$$y = z - H\hat{x}_{k|k-1}$$

The residual, or innovation, is the difference between what the sensor measured
and what the filter expected to measure from the predicted state:

- `z` is the actual measurement at time `k`.
- $H\hat{x}_{k|k-1}$ is the predicted measurement, found by mapping the
  predicted state into measurement space with `H`.
- `y` is the measurement error used by the update step. If `y` is close to zero,
  the prediction already matches the sensor well. If it is large, the correction
  step will pull the state estimate toward the new measurement.

Kalman gain

$$K = P H^T (H P H^T + R)^{-1}$$

The Kalman gain decides how strongly the new measurement should correct the
predicted state:

- `P` is the predicted state covariance. A larger `P` means the prediction is
  less certain, so the measurement should have more influence.
- `H P H^T` is the predicted uncertainty expressed in measurement space.
- `R` is the measurement noise covariance. A larger `R` means the sensor is less
  reliable, so the measurement should have less influence.
- `(H P H^T + R)^{-1}` normalizes the gain by the total expected measurement
  uncertainty.

In short, `K` is large when the prediction is uncertain and the sensor is
trusted, and small when the prediction is confident or the sensor is noisy.


Correct state:

$$\hat{x}_k = \hat{x}_{k|k-1} + Ky$$


Correct uncertainty

$$P_k = (I - KH)P$$


The corrected state starts with the predicted state $\hat{x}_{k|k-1}$ and adds
only the part of the innovation `y` that should be trusted. The Kalman gain `K`
decides how strongly the measurement pulls the prediction: a large gain moves
the estimate closer to the measurement, while a small gain keeps it closer to
the model prediction.

The corrected uncertainty is reduced by the information gained from the
measurement. `KH` describes how much of the predicted state uncertainty was
observed and corrected, so `(I - KH)` leaves only the uncertainty that remains
after applying the measurement update.


<details>
<summary>1d const velocity code</summary>
```
--8<-- "docs/Robotics/filter_and_estimator/kalman_filter/code/kalman_1d.py"
```
</details>


- F = how the system moves
- H = what you measure from the state
- P = how uncertain you are
- Q = how wrong the motion model may be
- R = how noisy the sensor is

---

## Reference
- [hummingbird - kalman filter](https://www.youtube.com/playlist?list=PLgG0XDQqJckmfolmM8y0GFS8l3x_r2p_S)
- [Why Use Kalman Filters? | Understanding Kalman Filters, Part 1](https://youtu.be/mwn8xhgNpFY?list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr)
- [Kalman Filter Part 1 — Introduction](https://medium.com/@mathiasmantelli/kalman-filter-series-introduction-6d2e2b28d4cf)
