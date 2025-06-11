---
tags:
    - opencv
    - cv_bridge
    - ros
    - custom build
---

# Build CV_Bridge for custom opencv build

Build [opencv 4.10](Programming/vision/opencv/build/) with cuda support
Use Docker to create build image

- clone github


## Build on docker
- docker base cuda **devel** with cudnn `FROM nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04` 
- Install dependencies
- Run colcon


```
sudo apt install
    libboost-all-dev \
    libboost-python-dev \
    python3-dev
```

```bash title="colcon"
colcon build --packages-up-to cv_bridge \
--cmake-args \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  -DCMAKE_INCLUDE_PATH=/usr/include/opencv4/
```

