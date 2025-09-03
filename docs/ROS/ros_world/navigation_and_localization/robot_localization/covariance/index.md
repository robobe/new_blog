---
title: Covariance 
tags:
    - fusing
    - sensor
    - covariance
---


{{ page_folder_links() }}

- **Covariance**: is the tendency for **two** variables to vary together, which is a way of being correlated!
- **Variance**: measures how much a single variable (e.g., sensor noise in one axis) fluctuates around its mean.
- **Bias**:  a systematic error, usually handled separately (e.g., calibration, bias states in the filter).


### Reference:
- [The Covariance Matrix : Data Science Basics](https://youtu.be/152tSYtiQbw)
---

## Demo: Calculate the covariance for optical velocity estimator

build the **covariance matrix** Σ that describes the uncertainty of (vx,vy)

### Step1: Collect samples
Collect estimate velocity for **N frames**

### Step2: Compute the mean velocity


$$
\mu_v = 
\begin{bmatrix}
\mu_{v_x} \\
\mu_{v_y}
\end{bmatrix}=\frac{1}{N}\sum_{i=1}^{N} v_i
$$

### Step3: Compute the deviations
For each measurement:
$$d_i = v_i - \mu_v$$

This gives you how much each velocity reading deviates from the mean.

### Step4: Build the covariance matrix

$$\Sigma =
\begin{bmatrix}
\text{Var}(v_x) & \text{Cov}(v_x,v_y) \\
\text{Cov}(v_y,v_x) & \text{Var}(v_y)
\end{bmatrix}$$

|   |   |
| :-- |---|
| $$\text{Var}(v_x) = \frac{1}{N-1}\sum (v_{x,i} - \mu_{v_x})^2$$  |   |
| $$\text{Var}(v_y) = \frac{1}{N-1}\sum (v_{y,i} - \mu_{v_y})^2$$  |   |
| $$\text{Cov}(v_x,v_y) = \frac{1}{N-1}\sum (v_{x,i} - \mu_{v_x})(v_{y,i} - \mu_{v_y})$$  |   |




### Code example

```python
import numpy as np

velocities = np.array([
    [1.0, 0.5],
    [1.1, 0.4],
    [0.9, 0.6],
    [1.2, 0.5]
])

# covariance (rows=variables, cols=samples)
cov_matrix = np.cov(velocities.T, bias=False)
print(cov_matrix)

```