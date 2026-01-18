---
title: Project AirSim
tags:
    - airsim
    - simulator
---


## Dev
- Dockerfile to install python dependencies 
- Install prebuild binary

```dockerfile
# syntax=docker/dockerfile:1
FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# --- OS deps (Python + GL runtime) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    python3 \
    python3-pip \
    python3-venv \
    # Fix for: libGL.so.1 not found
    libgl1 \
    # Common runtime libs often needed by OpenCV / GUI-ish deps
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

# --- Virtualenv ---
ENV VENV_PATH=/opt/venv
RUN python3 -m venv ${VENV_PATH}
ENV PATH="${VENV_PATH}/bin:${PATH}"

RUN pip install --no-cache-dir -U pip setuptools wheel

# --- Clone ProjectAirSim ---
ARG PROJECTAIRSIM_REF=main
WORKDIR /opt
RUN git clone --depth 1 --branch ${PROJECTAIRSIM_REF} https://github.com/iamaisim/ProjectAirSim.git

# --- Install the Python client ---
WORKDIR /opt/ProjectAirSim/client/python/projectairsim
RUN pip install --no-cache-dir -e .

# Sanity check
RUN python -c "import projectairsim; print('projectairsim import OK')"

WORKDIR /workspace
CMD ["bash"]
```

```sh title="docker usage"
docker run --rm -it --gpus all --net host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
airsim:22.04 /bin/bash
```

```
/opt/ProjectAirSim/client/python/example_user_scripts# python3 hello_drone.py
```

---

## Reference
- [iamai site](https://iamaisim.com/)
- [Project AirSim simulator](https://github.com/iamaisim/ProjectAirSim)
- [Project AirSim](https://iamaisim.github.io/ProjectAirSim/)