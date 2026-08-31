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
To convert onnx or other model format to rknn we use [github rknn-toolkit](https://github.com/airockchip/rknn-toolkit2/tree/master), the current version that we test for this post v2.3.2


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
For modern chips such as RK3566 and RK3588, the RKNN-Toolkit2 runs on the development PC and
provides:

- Model conversion to .rknn
- INT8 quantization
- Model validation
- Accuracy and performance analysis
- PC simulation or connected-board testing


### What to install

| Machine | Install | Purpose |
|---|---|---|
| Development PC | RKNN-Toolkit2 | Convert ONNX/PyTorch models into `.rknn` |
| Board using Python | RKNN-Toolkit-Lite2 | Load and run `.rknn` models |
| Board using C/C++ | RKNN Runtime, usually `librknnrt.so` | Native deployment and inference |
| Board firmware/kernel | RKNPU driver | Communicate with the NPU hardware |

!!! warning ""
    Do not install the complete RKNN-Toolkit2 package on the board. It belongs on the PC. Rockchip describes Lite2 as the
    board-side Python API and RKNN Runtime as the board-side C/C++ API. [Official architecture](https://github.com/airockchip/rknn-toolkit2/blob/master/README.md)

For C/C++ deployment, the board image often already contains the NPU driver and runtime. For Python deployment,
  install the matching Lite2 wheel from the Toolkit2 repository. Keep Toolkit2, Lite2/runtime, driver, and model
  **versions compatible**.


- [RKNN-toolkit2](https://github.com/airockchip/rknn-toolkit2)
- [RKNN toolkit lite2 (board packages)](https://github.com/airockchip/rknn-toolkit2/tree/master/rknn-toolkit-lite2/packages)

---

## PC

### install

Install python whell from [github airockchip/rknn-toolkit2](https://github.com/airockchip/rknn-toolkit2/tree/master/rknn-toolkit2/packages/x86_64)

### Convert model
[ultralytics yolo26](https://docs.ultralytics.com/integrations/rockchip-rknn)

```
yolo export model=yolo26n.pt format=rknn name=rk3588 opset=13
```

!!! warning "opset"
    ONNX has versions of its operator specification called opsets.

    An ONNX opset defines the behavior and available **versions** of those operators.


```
yolo26n.pt
    │
    │ Ultralytics + PyTorch
    ▼
yolo26n.onnx       ← opset=13 applies HERE
    │
    │ rknn-toolkit2
    ▼
yolo26n-rk3588.rknn
```

<div class="grid-container">
    <div class="grid-item">
        <a href="zoo/">
            <p>RKNN Model Zoo YOLOv8</p>
            <image src="images/zoo.png" width=150 height=150/>
        </a>
        <details>
            <summary>More...</summary>
            <p>
                Convert a pretrained YOLOv8 ONNX model for RK3566, copy it to
                the board, run RKNNLite inference, and understand the output.
            </p>
        </details>
    </div>
</div>

### Convert to INT8
INT8 quantization requires a representative calibration dataset. During RKNN compilation, representative images are passed through the network so RKNN Toolkit2 can determine quantization ranges/scales.

!!! info calibration images
    For the pretrained YOLO26n COCO model, you don't need a special set of “YOLO26 calibration images.” You need a collection of representative images similar to what the model will see during inference.

!!! tip coco8.yaml
    For a first test, the easiest choice is Ultralytics' small COCO dataset, coco8.yaml. Ultralytics' RKNN exporter accepts a dataset YAML via data=... and internally creates the image list that RKNN Toolkit2 uses for calibration.

```bash
uv run yolo export \
    model=yolo26n.pt \
    format=rknn \
    name=rk3566 \
    quantize=8 \
    data=coco8.yaml
```


---

## Board

```
uv venv --python 3.12

uv pip install \
https://raw.githubusercontent.com/airockchip/rknn-toolkit2/master/rknn-toolkit-lite2/packages/rknn_toolkit_lite2-2.3.2-cp312-cp312-manylinux_2_17_aarch64.manylinux2014_aarch64.whl

uv pip install opencv-python-headless
```

```bash title="check imports"
import cv2
import numpy as np

from rknnlite.api import RKNNLite
```

### Demo:

- [download bus image](code/bus.jpg)





#### Python

```python
import cv2
import numpy as np
from rknnlite.api import RKNNLite


MODEL = "yolo26n-rk3566.rknn"
IMAGE = "bus.jpg"

rknn = RKNNLite()

ret = rknn.load_rknn(MODEL)
if ret != 0:
    raise RuntimeError("Could not load RKNN")

ret = rknn.init_runtime()
if ret != 0:
    raise RuntimeError("Could not initialize RKNN runtime")


image = cv2.imread(IMAGE)

image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

image = cv2.resize(image, (640, 640))

input_tensor = np.expand_dims(image, axis=0)

outputs = rknn.inference(
    inputs=[input_tensor]
)

for i, output in enumerate(outputs):
    print(i, output.shape)

rknn.release()

```

```bash title="usage"
uv run python demo3.py
```

---

#### cpp

```bash
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    libopencv-dev
```

<details>
<summary>main.cpp</summary>
```
--8<-- "path/to/file.md"
```
</details>


<details>
<summary>CMakeLists.txt</summary>
```
--8<-- "path/to/file.md"
```
</details>


---
## Demo: run quantization models
!!! tip Yolo8 models
    Use convert models form
    [Qengineering / YoloV8-NPU](https://github.com/Qengineering/YoloV8-NPU/tree/main/rk3566)
    

!!! info hardware mapping
    | Rockchip SoC |              NPU | Typical Radxa products           | RKNN target       |
    | ------------ | ---------------: | -------------------------------- | ----------------- |
    | **RK3566**   |     ~1 TOPS INT8 | Zero 3W/3E, ROCK 3C, CM3/CM3S    | `rk3566`          |
    | **RK3568**   | ~0.8–1 TOPS INT8 | ROCK 3A, ROCK 3B, CM3I           | `rk3568` / RK356X |
    | **RK3576**   |  **6 TOPS INT8** | ROCK 4D, CM4, NX4                | `rk3576`          |
    | **RK3582**   |       **5 TOPS** | ROCK 5C Lite, E52C/E54C          | `rk3588`          |
    | **RK3588S**  |       **6 TOPS** | ROCK 5A, CM5, NX5                | `rk3588`          |
    | **RK3588**   |       **6 TOPS** | ROCK 5B/5B+, ROCK 5T, ROCK 5 ITX | `rk3588`          |    
    
<details>
<summary>Yolo8n example</summary>
```
--8<-- "docs/Embedded/other_boards/rockchip/rknn/code/8n/inference.py"
```
</details>

---


## Demo: hybrid-quantization

Quantize most of the YOLO26 network to INT8, but keep a few sensitive output layers in floating point, usually FP16.


---

#TODO: auto hybrid

### prerequisite

- yolo26n.onnx
- calibration.txt

**calibration.txt** contains representative images (i use the coco8 images for simplicity)

### convert code

<details>
<summary>Step1</summary>
```
--8<-- "docs/Embedded/other_boards/rockchip/rknn/code/hybrid/step1.py"
```
</details>


#### Edit yolo26n.quantization.cfg

- Replace
```
custom_quantize_layers: {}
```

- With

```
custom_quantize_layers:
  output0-rs: float16
  output0: float16
```

<details>
<summary>Step2</summary>
```
--8<-- "docs/Embedded/other_boards/rockchip/rknn/code/hybrid/step2.py"
```
</details>


#### Run and compare

<details>
<summary>Inference</summary>
```
--8<-- "docs/Embedded/other_boards/rockchip/rknn/code/hybrid/inference.py"
```
</details>


##### hybrid

```bash title="hybrid.model"
output[0]: shape=(1, 84, 8400), dtype=float32, min=0.0, max=671.5
class=5 confidence=0.884 bbox=(84, 130, 470, 319)
class=0 confidence=0.863 bbox=(220, 251, 80, 260)
class=0 confidence=0.853 bbox=(106, 239, 110, 300)
class=0 confidence=0.846 bbox=(464, 230, 91, 300)
class=0 confidence=0.482 bbox=(84, 328, 34, 190)
```

```bash title="fp16.mode"
output[0]: shape=(1, 84, 8400), dtype=float32, min=0.0, max=671.0
class=5 confidence=0.896 bbox=(85, 136, 470, 308)
class=0 confidence=0.855 bbox=(108, 235, 115, 299)
class=0 confidence=0.843 bbox=(212, 241, 73, 269)
class=0 confidence=0.827 bbox=(477, 229, 84, 292)
class=0 confidence=0.554 bbox=(79, 329, 37, 186)
Saved: bus_detected.jpg

```

---

## Reference
- [YOLO26 on RK3588; Hybrid INT8 Quantization (RKNN)](https://github.com/mahdieh-jokar/yolo26n-rknn-int8-quantization/blob/main/README.md)
- [YoloV8-NPU](https://github.com/Qengineering/YoloV8-NPU/tree/main)
- [Convert Custom Trained YOLO Models](https://docs.radxa.com/en/som/cm/cm3/app-development/ai/rknn-custom-yolo)


