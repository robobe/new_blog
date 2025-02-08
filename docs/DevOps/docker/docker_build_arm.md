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

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

The purpose of this command is to enable the host system to run binaries compiled for different architectures (such as ARM) using QEMU. This is particularly useful for building and running ARM Docker containers on an x86 host.

## Usage
```bash title="test"
docker run --platform=linux/arm64/v8 --rm -t arm64v8/ubuntu uname -m # Testing the emulation environment
```

!!! note "buildx"
     Check what is docker buildx and if preferred use it instead of qemu
--- 

## References
- [Running and Building ARM Docker Containers on x86](https://www.stereolabs.com/docs/docker/building-arm-container-on-x86)