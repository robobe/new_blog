---
title: Camera sensor
tags:
    - camera
    - sensor
---


{{ page_folder_links() }}


## focal length $ f_{x}, f_{y}$
Focal length is the distance from the lens’s optical center (or principal point) to the image sensor when the subject is in focus. in true pinhole camera $f_{x} = f_{y}$

$
f_{x} = \frac{\frac{W}{2}}{tan(\frac{hfov}{2})}
$

$
f_{y} = \frac{\frac{H}{2}}{tan(\frac{vfov}{2})}
$

## Principal Point $ c_{x}, c_{y}$
pixel location where the optical axis hits the sensor. Often close to the image center, but not necessarily exactly , $\frac{W}{2}, \frac{H}{2}$


## Intrinsic
Camera internal parameters

#### Minimal 
$
k=\begin{bmatrix}
 f & 0 & c_{x} \\
 0 & f & c_{y} \\
 0 & 0 & 1 \\
\end{bmatrix}
$

#### General 
$
k=\begin{bmatrix}
 f_{x} & s & c_{x} \\
 0 & f_{y} & c_{y} \\
 0 & 0 & 1 \\
\end{bmatrix}
$

## FOV (Field Of View)
The angular extent of the scene captured by the lens










