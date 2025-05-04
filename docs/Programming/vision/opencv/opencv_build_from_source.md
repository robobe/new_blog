---
tags:
    - opencv
    - build
    - cuda
---

# Build opencv from source with cuda support

## Tip: find CUDA Capability

using `deviceQuery` sample application

```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/NVIDIA/cuda-samples.git
cd cuda-samples
git sparse-checkout set Samples/1_Utilities/deviceQuery Common
```

```
cd cuda-samples/Samples/1_Utilities/deviceQuery/
mkdir build
cmake ..
make
```

```bash
./deviceQuery
#
 CUDA Device Query (Runtime API) version (CUDART static linking)

Detected 1 CUDA Capable device(s)

Device 0: "NVIDIA GeForce MX450"
  CUDA Driver Version / Runtime Version          12.6 / 11.5
  CUDA Capability Major/Minor version number:    7.5
  ...
```
