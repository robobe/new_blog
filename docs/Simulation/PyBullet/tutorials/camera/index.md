---
title: Pybullet camera
tags:
    - pybullet
    - camera
---

{{ page_folder_links() }}

<details>
    <summary>camera</summary>

```python
--8<-- "docs/Simulation/PyBullet/code/camera.py"
```
</details>


#### computeProjectionMatrixFOV
```
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
```

Creates a projection matrix for a virtual camera using a perspective (field-of-view) projection.  
**Parameters**:

- fov: Field of view in degrees (vertical angle).
- aspect: Aspect ratio (width/height).
- near: Near clipping plane distance.
- far: Far clipping plane distance.

**Result**:
Returns a 4x4 matrix that transforms 3D points into 2D camera space, simulating a real camera lens.
</details>

#### getCameraImage
```python
_, _, rgb, _, _ = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
```

Renders an image from the simulation as seen by the virtual camera.
**Parameters**:

- width, height: Output image size.
- view_matrix: Defines the camera’s position and orientation (where it looks from and to).
- projection_matrix: Defines how the 3D scene is projected onto the 2D image (from computeProjectionMatrixFOV).
- renderer: Rendering backend (here, OpenGL).

**Returns**:
A tuple containing image data, including the RGB image (rgb), depth, and segmentation masks.

![alt text](images/sim_camera.png)


<details>
    <summary>opencv view</summary>

```python
--8<-- "docs/Simulation/PyBullet/code/opencv.py"
```
</details>