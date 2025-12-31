---
title: Deepstream
tags:
    - Deepstream
---

```bash
docker run -it --rm --network=host \
  --runtime=nvidia \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
nvcr.io/nvidia/deepstream:8.0-samples-multiarch
```

```
gst-launch-1.0 videotestsrc is-live=true ! nvvideoconvert ! nveglglessink sync=false

```

```
version: "3.9"

services:
  gst:
    image: nvcr.io/nvidia/deepstream:8.0-samples-multiarch
    runtime: nvidia
    network_mode: host
    ipc: host
    shm_size: 1gb
    environment:
      DISPLAY: ${DISPLAY}
      NVIDIA_DRIVER_CAPABILITIES: all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /dev/shm:/dev/shm


```