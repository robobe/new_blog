---
tags:
    - opencv
    - cuda
---
# Opencv CUDA

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