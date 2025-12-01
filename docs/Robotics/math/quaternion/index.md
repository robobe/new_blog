---
title: Quaternion
tags:
    - math
    - quaternion
---
{{ page_folder_links() }}

Quaternions are an alternate way to describe **orientation** or **rotations** in 3D space using an ordered set of four numbers. They have the ability to uniquely describe any three-dimensional rotation about an arbitrary axis and do not suffer from gimbal lock


[Quaternions without math](https://youtu.be/1yoFjjJRnLY)
![alt text](image.png)


### Unit quaternion
A unit quaternion like unit vector is simply a vector with length (magnitude) equal to 1, but pointing in the same direction as the original vector.

quaternion:
$$q = w + xi + yj + zk$$

as a vector
$$q = (w, x, y, z)$$

calc the **norm**
$$\|q\| = \sqrt{w^2 + x^2 + y^2 + z^2}$$

calc **unit quaternion**
$$q_{unit} = \frac{q}{\|q\|} = \left(\frac{w}{\|q\|}, \frac{x}{\|q\|}, \frac{y}{\|q\|}, \frac{z}{\|q\|}\right)$$


<details>
    <summary>code example</summary>

```python
--8<-- "docs/Robotics/math/quaterniion/code/unit_quaternion.py"
```
</details>



---

### Conjugate
A quaternion is:
$$q = (x, y, z, w)$$

The conjugate is:
$$q^* = (-x, -y, -z, w)$$

!!! note "flip the signs of x, y, z, keep w the same."
     
---

### Slerp
**S**pherical **L**inear int**ERP**olation (SLERP) is a method to smoothly interpolate between two orientations (unit quaternions).
- SLERP traces the shortest arc along that sphere between them


$$SLERP(q_0, q_1, t) = \frac{\sin((1-t)\theta)}{\sin(\theta)} q_0 + \frac{\sin(t\theta)}{\sin(\theta)} q_1$$

- At t=0 → result = q0
- At t=1 → result = q1
- At t=0.5 → halfway rotation between them.

<details>
    <summary>Demo: using scipy</summary>

```python
--8<-- "docs/Robotics/math/quaterniion/code/slerp.py"
```
</details>



---

### Inverse

!!! tip "unit quaternion"
     If q is a unit quaternion (∥q∥=1), then the **conjugate** is also the **inverse**.


when not unit length

$$q^{-1} = \frac{q^*}{\|q\|^2}$$

$$q \cdot q^{-1} = 1$$

---

### Multiple
#### by vector

$$\mathbf{v}' = q \, \mathbf{v} \, q^{-1}$$

we treat the vector as pure quaternion

$$\mathbf{v} = (v_x, v_y, v_z, 0)$$


<details>
    <summary>Numpy Example</summary>

```python
--8<-- "docs/Robotics/math/quaterniion/code/multiple.py"
```
</details>


#### by quaternion

Quaternion multiplication is used to combine rotations in 3D space

$$q_1 = (w_1, x_1, y_1, z_1), \quad q_2 = (w_2, x_2, y_2, z_2)$$

$$q = q_2 \cdot q_1 = 
\Big( 
w_2 w_1 - x_2 x_1 - y_2 y_1 - z_2 z_1, \;
w_2 x_1 + x_2 w_1 + y_2 z_1 - z_2 y_1, \;
w_2 y_1 - x_2 z_1 + y_2 w_1 + z_2 x_1, \;
w_2 z_1 + x_2 y_1 - y_2 x_1 + z_2 w_1
\Big)$$


!!! warning 
    $$q_2 q_1 \neq q_1 q_2$$
     
---

## Reference
[watch again](https://youtu.be/jTgdKoQv738)