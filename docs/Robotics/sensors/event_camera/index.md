---
title: Event Camera
tags:
    - event camera
---


{{ page_folder_links() }}

## Resources

- [Secrets of Event-Based Optical Flow (T-PAMI 2024, ECCV 2022)](https://github.com/tub-rip/event_based_optical_flow)
    - [The MVSEC Dataset ](https://daniilidis-group.github.io/mvsec/)


## Dataset MVSEC

[Download](https://daniilidis-group.github.io/mvsec/download/)

```bash
pip install h5py
```

```python
--8<-- "docs/Robotics/sensors/event_camera/code/dataset_iteration.py"
```

1. *_data.hdf5
This is the raw sensor data:

2. *_gt.hdf5
This is the ground truth file:


### View image from file
```python
--8<-- "docs/Robotics/sensors/event_camera/code/view_data.py"
```



---

## Resources
- ["Event-based Robot Vision" Course taught at TU Berlin, Germany, during Spring 2020.](https://sites.google.com/view/guillermogallego/teaching/event-based-robot-vision)