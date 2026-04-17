---
title: SDF frames and pose      
tags:
    - sdf
    - frames
    - pose
    - link
    - joint
---

!!! tip "frame"
    a coordinate system in space (pose and orientation)


## model
- model frame 
- link frame
- joint frame
- custom frame


### model frame
The root frame in model used as a default reference frame

```xml
<model name="box">
  <pose>...</pose>  <!-- model frame -->
```

### link frame

```xml
<link name="link">
  <pose>...</pose>
```

Where the link relative to (default model frame)

### joint frame

```xml
<joint>
  <pose>...</pose>
```

### custom frame
- Just a named coordinate system
- Does NOTHING unless used as reference by `relative_to`
```xml
<frame name="box_frame">
  <pose>0 0 1.5 0 0 0</pose>
</frame>
```

!!! info "default behavior"
    if we don't specify `pose` it relative to **model frame** for links, joints, frames
    
    