---
title: State Space Model
tags:
    - control
    - state space
---


{{ page_folder_links() }}

[Good Video - Intro to Control - 6.1 State-Space Model Basics ](https://youtu.be/g9G8b7FxEHc)

$$\dot{x} = A x + B u$$

Where:

- x = state vector (what describes the system right now)
- u = control input (what you apply)
- $\dot{x}$ = how the state is changing
- A = system (physics) matrix
- B = input (control) matrix

### A matrix
It describes the natural physics of the system.

**A tells how the system changes by itself, without any control input.**

---

## Demo: Mass on a frictionless surface

Physics equation

$$m \ddot{x} = u$$ (f=ma)

### Define the state

$$x_1 = x \quad \text{(position)}$$
$$x_2 = \dot{x} \quad \text{(velocity)}$$

The state vector is:
$$x =
\begin{bmatrix}
x \\
\dot{x}
\end{bmatrix}$$


---

## Reference
- [Introduction to State-Space Equations | State Space, Part 1](https://youtu.be/hpeKrMG-WP0)
- [How To Derive A State Space Model For A Mobile Robot](https://abbhicse.medium.com/how-to-derive-a-state-space-model-for-a-mobile-robot-f19db53ffb30)