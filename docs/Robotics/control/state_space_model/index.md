---
title: State Space Model
tags:
    - control
    - state space
---


{{ page_folder_links() }}
A State Space Model is linear representation of dynamic system,

[Good Video - Intro to Control - 6.1 State-Space Model Basics ](https://youtu.be/g9G8b7FxEHc)

$$\dot{x} = A x + B u$$

Where:

- x = state vector (what describes the system right now) ($R^{n}$)
- u = control input (what you apply) ($R^{m}$)
- $\dot{x}$ = how the state is changing
- A = system (physics) matrix ($R^{n*n}$)
- B = input (control) matrix ($R^{n*m}$)

### A matrix
It describes the natural physics of the system.

**A tells how the system changes by itself, without any control input.**

---

## Demo: mass on a spring system

$$
u(t) - kx(t)-b\dot{x}(t) = m\ddot{x}(t)
$$

- u(t) : Input force
- kx(t): Spring force
- $b\dot{x}$: frication
- $m\ddot{x}(t)$: ma

### Named variables

- $x(t) = x_{1}$
- $\dot{x}(t) = x_{2} = \dot{x_1}$
- $u(t) = u_1$

$$
u_1 - kx_1 - bx_2 = m\dot{x_2}
$$

find dynamic

- $\dot{x_1}= ?$
- $\dot{x_2}= ?$


$\dot{x_1} = x_2$

$\dot{x_2} = \frac{1}{m}u_1 - \frac{k}{m}x_1 - \frac{b}{m}x_2$

### rewrite the equation 
related to $x_1, x_2, u_1$

$\dot{x_1} = 0x_1 + 1x_2 + 0u_1$

$\dot{x_2} = -\frac{k}{m}x_1 - \frac{b}{m}x_2 + \frac{1}{m}u_1$

### rewrite as a matrix
$$
\begin{bmatrix}
\dot{x_1} \\
\dot{x_2}
\end{bmatrix}
=\begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{b}{m}
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}
+
\begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix}
u_1
$$

$$\dot{x} =
\begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{b}{m}
\end{bmatrix}
x
+
\begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix}
u$$

$$\dot{x} = A x + B u$$


## Output
How is the ouput (x) depend and the system state and control input

in the example we want to find change in distance $x(t)$

- $y = x_1 = x(t)$


$y = Cx + Du$



$$
y = 1x_1 + 0x_2 + 0u_1
$$

$$y =
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}
+
\begin{bmatrix}
0
\end{bmatrix}
u_1$$

for example if we want to look at the system if the velocity is the output $y = \dot(x) = x_2$

$$
y = 0x_1 + 1x_2 + 0u_1
$$

$$y =
\begin{bmatrix}
0 & 1
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}
+
\begin{bmatrix}
0
\end{bmatrix}
u_1$$

---

## Reference
- [Introduction to State-Space Equations | State Space, Part 1](https://youtu.be/hpeKrMG-WP0)
