---
title: VulkanSceneGraph
tags:
    - vulkan
    - vsg
---


{{ page_folder_links() }}

## Install
- Install on ubuntu 22.04
- Install with glslang optional (ver 14.0.0)

!!! warning "glslang-dev glslang-tools"
    Only if ubuntu 24.04 
    for ubuntu 22.04 install from source
     
```
sudo apt-get install cmake-curses-gui g++ git libvulkan-dev \
glslang-dev glslang-tools
```

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

### glslang
- Install version 15.0 (vsg complain on version 16.0)
- 
```bash
git checkout tags/15.0.0
cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_OPT=off
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```