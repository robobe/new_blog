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

**Equation 1**
$\dot{x_1} = 0x_1 + 1x_2 + 0u_1$

**Equation 2**
$\dot{x_2} = -\frac{k}{m}x_1 - \frac{b}{m}x_2 + \frac{1}{m}u_1$

### rewrite as a matrix
#### Matrix A

$A =
\begin{bmatrix}
\text{coeff of } x_1 \text{ in eq1} &amp; \text{coeff of } x_2 \text{ in eq1} \\
\text{coeff of } x_1 \text{ in eq2} &amp; \text{coeff of } x_2 \text{ in eq2}
\end{bmatrix}
$

so:

$A =
\begin{bmatrix}
0 &amp; 1 \\
-\frac{k}{m} &amp; -\frac{b}{m}
\end{bmatrix}
$

#### Matrix B

$B =
\begin{bmatrix}
\text{coeff of } u \text{ in eq1} \\
\text{coeff of } u \text{ in eq2}
\end{bmatrix}
$

so:

$B =
\begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix}
$

### Final

$$\dot{x} = A x + B u$$

$
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
$

#### Write as a vector state

$\dot{x} =
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
u
$

---


## Output
State-space has two parts

$$\dot{x} = Ax + Bu \quad \text{(system dynamics)}$$

$$y = Cx + Du \quad \text{(what you measure / output)}$$

👉 A, B → physics
👉 C, D → what you choose to observe

### Demo: Mass String system

State:

$$
x =
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}
=\begin{bmatrix}
position \\
velocity
\end{bmatrix}
$$


#### Case 1: Output = position

!!! tip 
    C is just a matrix that **picks** which part of the state you want to **observe**.

##### Step1

I want output = position

$$y = x_1$$

but the state vector 
$$x =
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix}$$

##### Step 2
Build **C** matrix as selector

$$y =
\begin{bmatrix}
1 & 0
\end{bmatrix}
x$$


##### Step 3
Define **D**

!!! tip "Ask Chat"
    Position depends on control input, so shouldn’t $ D\ne0$ ?”
    

$$D = 0$$


##### Final
$$y = Cx + Du$$
$$y =
\begin{bmatrix}
1 & 0
\end{bmatrix}
x + 0 \cdot u$$

---

## Reference
- [Introduction to State-Space Equations | State Space, Part 1](https://youtu.be/hpeKrMG-WP0)
