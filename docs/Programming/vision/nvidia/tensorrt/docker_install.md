---
tags:
    - embedded
    - nvidia
    - jetson
    - orin
    - tensorrt
---

# TensorRT

## Install

!!! tip 
     To download tensorrt we must login to nvidia



- download local repo package
- install the package
- register the gpg key
- run `apt update`
- install `apt tensorrt`



- [Download from nvidia](https://developer.nvidia.com/tensorrt/download/10x)

![alt text](images/nvidia_tensorrt_download_page.png)

```bash title="Install TensorRT"
dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5_1.0-1_arm64.deb 
cp /var/nv-tensorrt-local-repo-ubuntu2204-10.3.0-cuda-12.5/nv-tensorrt-local-F9A70CFC-keyring.gpg /usr/share/keyrings/
apt update
apt install tensorrt
# check
dpkg -l | grep tensorrt

```


```python
import tensorrt as trt

logger = trt.Logger(trt.Logger.WARNING)
runtime = trt.Runtime(logger)

with open("model.engine", "rb") as f:
    engine = runtime.deserialize_cuda_engine(f.read())

```

![](/assets/images/under_construction.png)


---

## Reference
[TensorRT In Docker](https://leimao.github.io/blog/Docker-TensorRT/)