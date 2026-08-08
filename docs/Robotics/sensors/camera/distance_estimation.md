---
title: Distance Estimation from a Known Object Size
tags:
    - computer-vision
    - camera
    - distance-estimation
    - calibration
---

Estimate the distance to an object from one image when one real object
dimension is known. The method uses the pinhole-camera model and the measured
size of that dimension in pixels.

!!! warning "This is an estimate"
    Accuracy depends on camera calibration, lens distortion, object orientation,
    and the quality of the pixel measurement. Do not use this method by itself
    for safety-critical ranging.

## Basic formula

For a known real-world width:

$$
Z = \frac{S_{real} f_x}{S_{pixels}}
$$

Where:

- $Z$ is depth along the camera optical axis;
- $S_{real}$ is the known object width in metres;
- $S_{pixels}$ is the observed object width in pixels;
- $f_x$ is the horizontal focal length in pixels.

This calculates optical-axis depth, not necessarily the straight-line distance
to an object far from the image center.

For a known object height, use its height in pixels and the vertical focal
length $f_y$ instead:

$$
Z = \frac{H_{real} f_y}{H_{pixels}}
$$

!!! tip "Match the measurement direction"
    Use $f_x$ with widths and $f_y$ with heights. Do not mix a horizontal focal
    length with a vertical pixel measurement.

---

## Obtain the focal length

### From camera calibration

Calibration is the preferred source. The intrinsic matrix contains both focal
lengths:

$$
K = \begin{bmatrix}
f_x & s & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

Use `K[0, 0]` as $f_x$ or `K[1, 1]` as $f_y$. The calibration must match the
resolution used for distance estimation. Scale the intrinsic parameters if the
image was resized after calibration.

### From field of view

If calibration data is unavailable, estimate $f_x$ from the horizontal field
of view:

$$
f_x = \frac{W_{image}}{2\tan(FOV_x/2)}
$$

Where $W_{image}$ is the image width in pixels and $FOV_x$ is the horizontal
field of view. The angle must be converted to radians before using it in a
programming-language trigonometric function.

Combining the focal-length and distance equations gives:

$$
Z = \frac{S_{real}W_{image}}
         {2S_{pixels}\tan(FOV_x/2)}
$$

Advertised FOV values may be approximate. A calibrated $f_x$ normally produces
better results.

---

## Worked example

Suppose:

- image resolution: $1920 \times 1080$;
- horizontal FOV: $90^\circ$;
- real object width: $0.5$ m;
- measured object width: $200$ pixels.

First calculate the focal length:

$$
f_x = \frac{1920}{2\tan(45^\circ)} = 960\text{ pixels}
$$

Then calculate the distance:

$$
Z = \frac{0.5 \times 960}{200} = 2.4\text{ m}
$$

The estimated optical-axis distance is `2.4 m`.

---

## Python example

[Download `distance_estimation.py`](code/distance_estimation.py)

```python title="distance_estimation.py"
--8<-- "docs/Robotics/sensors/camera/code/distance_estimation.py"
```

Output:

```text
Estimated distance: 2.40 m
```

When calibrated $f_x$ is already available, the calculation is simply:

```python
distance_m = real_width_m * fx_pixels / object_width_px
```

---

## Measuring the object in the image

The pixel width may come from:

- manually selected edge points;
- an OpenCV contour or marker detector;
- the corners of an ArUco or AprilTag marker;
- an object detector's bounding box.

Detector bounding boxes are convenient but often change with confidence,
occlusion, and object rotation. A planar marker with known corner geometry is
usually more repeatable.

If the image is resized, either measure the object in the calibrated resolution
or scale $f_x$ by the same horizontal factor:

$$
f_{x,new} = f_{x,calibration}\frac{W_{new}}{W_{calibration}}
$$

---

## Sources of error

- **Object rotation:** a rotated object appears narrower, so the estimated
  distance becomes too large.
- **Lens distortion:** measurements near the image edge can be biased unless
  the image is rectified or distortion is included in the model.
- **Incorrect object size:** objects in the same detected class may have
  different physical dimensions.
- **Loose bounding boxes:** a few pixels of error matter increasingly as the
  object becomes small and distant.
- **Wrong focal length:** FOV specifications, digital cropping, zoom, focus,
  and image resizing can change the effective value.
- **Non-planar objects:** a bounding box may contain points at different depths.

!!! tip "Improve practical accuracy"
    Calibrate the camera at the working resolution, rectify the image, use a
    clearly defined physical dimension, and validate the result at several
    measured distances. For a planar target with known feature points, pose
    estimation with `solvePnP` is usually more robust than one-dimensional size
    estimation.

## Reference

- [Estimating Camera Distance — shared ChatGPT conversation](https://chatgpt.com/share/6a74dd62-1614-83eb-b643-6fb6b6fc82b1){:target="_blank" rel="noopener noreferrer"}

<!-- post-content-skill: 1.0.0 -->
