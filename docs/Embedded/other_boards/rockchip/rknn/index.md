---
title: RKNN
tags:
    - radxa
    - rocky
    - rknn
    - yolo
---

## What is RKNN?
RKNN is Rockchip’s model format for running neural networks on Rockchip NPUs.



###  Typical workflow:

```
  PyTorch / ONNX model
          ↓
  RKNN-Toolkit2 on PC
          ↓
  model.rknn
          ↓
  RKNN Runtime on Rockchip board
          ↓
  NPU inference
```


## What is RKNN-Toolkit?
For modern chips such as RK3566, RK3568, RK3576, and RK3588, use RKNN-Toolkit2. It runs on the development PC and
provides:

- Model conversion to .rknn
- INT8 quantization
- Model validation
- Accuracy and performance analysis
- PC simulation or connected-board testing


### What to install

   Machine                  Install                               Purpose
  ━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Development PC           RKNN-Toolkit2                         Convert ONNX/PyTorch models into .rknn
  ───────────────────────  ────────────────────────────────────  ────────────────────────────────────────
   Board using Python       RKNN-Toolkit-Lite2                    Load and run .rknn models
  ───────────────────────  ────────────────────────────────────  ────────────────────────────────────────
   Board using C/C++        RKNN Runtime, usually librknnrt.so    Native deployment and inference
  ───────────────────────  ────────────────────────────────────  ────────────────────────────────────────
   Board firmware/kernel    RKNPU driver                          Communicate with the NPU hardware

!!! warning ""
    Do not install the complete RKNN-Toolkit2 package on the board. It belongs on the PC. Rockchip describes Lite2 as the
    board-side Python API and RKNN Runtime as the board-side C/C++ API. [Official architecture](https://github.com/airockchip/rknn-toolkit2/blob/master/README.md)

For C/C++ deployment, the board image often already contains the NPU driver and runtime. For Python deployment,
  install the matching Lite2 wheel from the Toolkit2 repository. Keep Toolkit2, Lite2/runtime, driver, and model
  versions compatible.


[rknn-toolkit2](https://github.com/airockchip/rknn-toolkit2)


## Reference
- [YOLO26 on RK3588; Hybrid INT8 Quantization (RKNN)](https://github.com/mahdieh-jokar/yolo26n-rknn-int8-quantization/blob/main/README.md)
- [YoloV8-NPU](https://github.com/Qengineering/YoloV8-NPU/tree/main)


```
uv venv --system-site-packages
```