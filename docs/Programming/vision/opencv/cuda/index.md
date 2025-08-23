---
title: OpenCV CUDA
tags:
    - opencv
    - cuda
---

{{ page_folder_links() }}

## Check

Check if openCV compiled with cuda support

```python
print("CUDA support:", cv2.cuda.getCudaEnabledDeviceCount() > 0)
```

## Simple
Simple python script that generate image upload to cuda , run alogithim from cv2 suda namespace and download back to cpu usage

```python title="cuda upload, run , download"
--8<-- "docs/Programming/vision/opencv/cuda/code/simple.py"
```

---

## Optimize Upload / Download memory transfer cpu/gpu

### cv2.cuda.HostMem
using cv2.cuda.HostMem lets you allocate page-locked (pinned) memory in host RAM, which speeds up transfers between CPU ↔ GPU
- Normally, NumPy arrays are allocated in pageable memory → GPU transfers require an extra copy step.
- With HostMem, memory is allocated as page-locked (pinned) memory, which can be directly DMA-transferred to the GPU → faster upload/download.

- PAGE_LOCKED (1)→ pinned memory (fastest transfers)
- SHARED (2)→ memory accessible by both CPU and GPU
- WRITE_COMBINED (4) → faster host-to-device writes, slower reads

!!! warning "python opencv const not expose use integers instead "
     

```python
--8<-- "docs/Programming/vision/opencv/cuda/code/pin_memory.py"
```

!!! note "createMatHeader"
    A HostMem buffer by itself isn’t directly usable as a NumPy array. thy are buffer in host RAM
    call .createMatHeader(), which returns a cv::Mat header that views into the same memory. (wrap the buffer)
    and return the buffer as ndarray

---
## Build dev environment using vscode devcontainer
- Using the post from [Build OpenCV 4.10 with cuda](/Programming/vision/opencv/build/)
- We Got `OpenCV-unknown-aarch64-*` from ARM (Jetson) and `OpenCV-unknown-x86_64-*`


```dockerfile
FROM nvidia/cuda:12.6.0-cudnn-runtime-ubuntu22.04

COPY opencv_debs/* /tmp/opencv_debs/
RUN cd /tmp/opencv_debs && \
    apt update && \
    ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ]; then \
        apt install -y ./OpenCV-unknown-aarch64-*.deb; \
    elif [ "$ARCH" = "x86_64" ]; then \
        apt install -y ./OpenCV-unknown-x86_64-*.deb; \
    else \
        echo "Unsupported architecture: $ARCH"; exit 1; \
    fi && \
    rm -rf /tmp/opencv_debs
```

## Reference
- [Build OpenCV 4.10 with cuda](/Programming/vision/opencv/build/)
- [OpenCV CUDA integration](https://www.simonwenkel.com/notes/software_libraries/opencv/opencv-cuda-integration.html)