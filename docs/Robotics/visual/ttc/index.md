---
title: Visual Time to Collision (TTC) Geometry
tags:
    - robotics
    - vision
    - ttc
    - collision-avoidance
---

# Visual Time to Collision (TTC)

**Time to collision (TTC)** is the estimated time until an approaching object
reaches the camera, assuming its current relative velocity stays constant.

A camera can estimate TTC without knowing the object's real size or distance.
The key observation is **looming**: an object appears larger in the image as it
gets closer.

!!! warning
    A small TTC does not by itself prove that a collision will occur. The
    object's path must also overlap the camera or robot's path.

## 1. Start with pinhole-camera geometry

For an object facing the camera:

- \(W\): real object width;
- \(Z\): distance from the camera;
- \(f\): camera focal length in pixels;
- \(w\): object width measured in the image, in pixels.

Similar triangles give:

\[
\frac{w}{f} = \frac{W}{Z}
\]

Therefore:

\[
w = \frac{fW}{Z}
\]

The real width \(W\) and focal length \(f\) are constant. When distance \(Z\)
decreases, image width \(w\) increases.

## 2. Relate image growth to TTC

Differentiate \(w=fW/Z\) with respect to time:

\[
\dot{w} = -\frac{w\dot{Z}}{Z}
\]

For an approaching object, \(\dot{Z}<0\). TTC is distance divided by closing
speed:

\[
TTC = \frac{Z}{-\dot{Z}}
\]

Substituting the image-growth equation gives:

\[
\boxed{TTC = \frac{w}{\dot{w}}}
\]

The unknown object width and camera focal length cancel. Only the object's
image size and growth rate are needed.

## 3. Estimate TTC from two frames

Let:

- \(w_0\) be the measured width in the first frame;
- \(w_1\) be the width in the second frame;
- \(\Delta t\) be the time between frames.

Assuming constant closing velocity, the TTC at the second frame is:

\[
\boxed{TTC_1 = \Delta t\frac{w_0}{w_1-w_0}}
\]

This two-frame form comes directly from \(Z=fW/w\), so \(fW\) still cancels.

If \(w_1 \le w_0\), the object is not growing in the image. Do not report a
finite approaching TTC from these two measurements.

## Simple example

An object's bounding-box width changes from \(80\) pixels to \(100\) pixels in
\(0.1\) seconds:

\[
w_0=80, \qquad w_1=100, \qquad \Delta t=0.1\text{ s}
\]

Apply the two-frame formula:

\[
TTC_1 = 0.1\frac{80}{100-80}
      = 0.1\frac{80}{20}
      = 0.4\text{ s}
\]

At the time of the second frame, the object is estimated to be \(0.4\) seconds
from the camera plane if the same motion continues.

## Interactive simulation

Press **Start** to watch an object approach the camera at a constant \(2\) m/s.
The side view shows the decreasing distance. The camera view shows the same
object expanding according to \(w=fW/Z\).

<iframe
    src="code/ttc_demo.html"
    width="100%"
    height="560"
    frameborder="0"
    loading="lazy">
</iframe>

The actual value is \(Z/v\). Every \(0.25\) seconds, the demo measures two
projected widths and calculates
\(TTC_1=\Delta t\,w_0/(w_1-w_0)\). After the second sample, the estimated and
actual TTC should closely match. The dashed rectangle marks the latest sampled
image width.

## Practical steps

1. Detect or track the same object in consecutive frames.
2. Measure the same size quantity in both frames, such as bounding-box width.
3. Calculate \(\Delta t\) from the frame timestamps, not only the nominal frame rate.
4. Confirm that the measured size increased.
5. Calculate \(TTC_1=\Delta t\,w_0/(w_1-w_0)\).
6. Combine TTC with object position or predicted path before declaring a
   collision risk.

If object area \(A\) is more stable than width, use the linear scale
\(w=\sqrt{A}\) in the same formula.

## Assumptions and limitations

The simple geometry assumes:

- constant relative velocity between the two frames;
- the same rigid object is tracked in both frames;
- object orientation and visible shape do not change significantly;
- image-size growth is caused mainly by motion toward the camera;
- the camera and timestamps are stable enough to measure the growth.

Bounding-box jitter, rotation, partial occlusion, camera motion, and very small
changes in image size can make TTC noisy. Real collision-avoidance systems
normally smooth several measurements and combine visual TTC with trajectory or
range information.

---


## Reference

- [time to collision using optical flow](https://github.com/KennethYangle/time-to-collision)
- [Motion Modeling in Videos: Part 2 — Optical Expansion](https://medium.com/@d.d.tananaev/motion-modeling-in-videos-part-2-optical-expansion-816abe1e92d9)