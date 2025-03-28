---
tags:
    - nvidia
    - jetson
    - docker
---

# Docker nvidia

[Nvidia NGC Catalog](https://catalog.ngc.nvidia.com/containers)

|   |   |   |
|---|---|---|
| [![alt text](images/jetpack.png)](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack)  | [![alt text](images/cross.png)](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/jetpack-linux-aarch64-crosscompile-x86)  | [![alt text](images/l4t_cuda.png)]()  |


## Jetpack

```
docker pull nvcr.io/nvidia/l4t-jetpack:r36.4.0
```
## Nvidia container runtime

```bash
sudo apt install nvidia-container-toolkit
```

```bash title="usage"
sudo docker run --gpus all --runtime=nvidia -it --rm nvcr.io/nvidia/l4t-jetpack:r36.4.0 nvidia-smi
```

                                                                            