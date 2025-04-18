---
tags:
    - docker
    - arm
    - qemu
---

# Running and Building ARM Docker Containers on x86


## Install

```bash 
sudo apt-get install qemu binfmt-support qemu-user-static
```

!!! note "binfmt"
     binfmt (Binary Format) is a Linux kernel feature that allows execution of non-native binary formats (executables compiled for different architectures). This enables cross-architecture execution, such as running ARM64 binaries on an x86_64 machine.

    binfmt works with interpreters like **QEMU** to translate and execute foreign binaries transparently.

## Registration With docker

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

!!! note "-p yes"
     This register qemu persistent and available for future docker to run

!!! note "multiarch/qemu-user-static"
    - The **multiarch/qemu-user-static** image is pulled and run temporarily to register QEMU with your system.
    - It doesn't leave behind an actual image that appears in docker images.
    - 
The purpose of this command is to enable the host system to run binaries compiled for different architectures (such as ARM) using QEMU. This is particularly useful for building and running ARM Docker containers on an x86 host.

## Usage
```Dockerfile title="Dockerfile"

FROM ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*
CMD ["/bin/bash"]
```

### Build and Run
```bash title="Build"
docker build --platform linux/arm64 -t ubuntu/22.04:arm64 .
docker build --platform linux/amd64 -t ubuntu/22.04:amd64 .
```

```bash title="Run"
docker run --rm -it --platform linux/arm64 ubuntu/22.04:arm64 uname -m
docker run --rm -it --platform linux/amd64 ubuntu/22.04:amd64 uname -m
```

!!! note "buildX"
    Use buildX to build multi-arch images with the same TAG work only if we push to a registry (Manifest)

   
     
--- 

## References
- [Running and Building ARM Docker Containers on x86](https://www.stereolabs.com/docs/docker/building-arm-container-on-x86)