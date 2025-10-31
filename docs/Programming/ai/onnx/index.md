---
tags:
  - onnx
  - ai
---

# ONNX

ONNX (Open Neural Network Exchange) is an open-source format for representing machine learning models, designed to enable interoperability between different deep learning frameworks

<div class="grid-container">
    <div class="grid-item">
        <a href="cpp">
        <p>C++</p></a>
    </div>
    <div class="grid-item">
        <a href="onnx">
        <p>---</p></a>
    </div>
    <div class="grid-item">
        <a href="---">
        <p>---</p></a>
    </div>
     
</div>

---

# #TODO: move to it's on section

## Install on nvidia jetson with jetpack 6.2

!!! warning "onnx version" - The last package from [Jetson Zoo](https://www.elinux.org/Jetson_Zoo#ONNX_Runtime) is for jetpack 6.0 - I found the last version 1.23 [pypi.jetson-ai-lab.io](https://pypi.jetson-ai-lab.io/jp6/cu126/+f/e1e/9e3dc2f4d5551/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl) it's installed successfully

!!! warning "pypi.jetson-ai-lab.io"
many sites refer to nvidia url: https://pypi.jetson-ai-lab.dev/jp6/cu126, note to the **dev** suffix

    the `dev` part reploace by `io`
    https://pypi.jetson-ai-lab.io/jp6/cu126


### Installed on docker

Install onnxruntime-gpu on docker that run on jetson orin using vscode and devcontainer

```
.
|-- .devcontainer
|   |-- Dockerfile
|   `-- devcontainer.json
|-- .gitignore
|-- .vscode
|-- README.md
|-- docker-compose.yaml
|-- requirements.txt
`-- scripts
    `-- check.py
```

!!! tip ""
At last i update my host with the command `sudo apt install nvidia-jetpack`
And use the docker image **nvcr.io/nvidia/l4t-jetpack:r36.4.0**
find at [NGC Catalog](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack/)

| Package | version | note |
| ------- | ------- | ---- |
| jetpack |  R36.4.3       |  cat /etc/nv_tegra_release     | 
| cuda    |   2.6      |  nvcc --version     |
| cudnn   |  9.3.0       |  `dpkg -l \| grep cudnn`     |
| tensorrt  | 10.3.0.30        |  `dpkg -l \| grep tensorrt`    |
| onnxruntime |  1.23.0       |  ```python3 -c "import onnxruntime as ort; print('ONNX Runtime version:', ort.__version__)```"     |

---

## Code

<details>
    <summary>.devcontainer/devcontainer.json</summary>

```json
--8<-- "docs/Programming/ai/onnx/code/.devcontainer/devcontainer.json"
```

</details>

<details>
    <summary>.devcontainer/Dockerfile</summary>

```dockerfile
--8<-- "docs/Programming/ai/onnx/code/.devcontainer/Dockerfile"
```

</details>

<details>
    <summary>docker-compose.yaml</summary>

```yaml
--8<-- "docs/Programming/ai/onnx/code/docker-compose.yaml"
```

</details>

<details>
    <summary>requirements.txt</summary>

```txt
--8<-- "docs/Programming/ai/onnx/code/requirements.txt"
```

</details>


!!! tip "python package location"
     https://pypi.jetson-ai-lab.io/jp6/cu129

### Run

```python
import onnxruntime as ort

ort.set_default_logger_severity(0)  # 0 = VERBOSE, 1 = INFO, 2 = WARNING, 3 = ERROR, 4 = FATAL


# Show all available providers on your machine
print("Available providers:", ort.get_available_providers())
print("ONNX Runtime version:", ort.__version__)
print("Build info:", ort.get_device())

```

```
Available providers: ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
ONNX Runtime version: 1.20.0
Build info: GPU
```

---

## Demo: mnist

[MNIST model home page](https://github.com/onnx/models/tree/main/validated/vision/classification/mnist)

[mnist-8.onnx](https://github.com/onnx/models/raw/refs/heads/main/validated/vision/classification/mnist/model/mnist-8.onnx)

!!! warning ""
[W:onnxruntime:Default, device_discovery.cc:164 DiscoverDevicesForPlatform] GPU device discovery failed: device_discovery.cc:89 ReadFileContents Failed to open file: "/sys/class/drm/card1/device/vendor"

    for known i don't found and reference that help resolve the issue


- work with onnxruntime-gpu version 1.23
- need numpy <2
- images [zero](code/images/zero.png), [six](code/images/six_1.png)
- model: [mnist-8.onnx](code/model/mnist-8.onnx)
- try with the 3 providers
  - on the docker side success only with the cpu (TODO)
  - on the host all off them work

```python
import onnxruntime as ort
import numpy as np
import cv2

ort.set_default_logger_severity(0)  # 0 = VERBOSE, 1 = INFO, 2 = WARNING, 3 = ERROR, 4 = FATAL


# --- Load ONNX model ---
# "CPUExecutionProvider"
# CUDAExecutionProvider
# TensorrtExecutionProvider
session = ort.InferenceSession("model/mnist-8.onnx", providers=["CUDAExecutionProvider"])

# --- Load and preprocess an image (28×28 grayscale) ---
# You can replace 'digit.png' with your own file.
img = cv2.imread("images/six_1.png", cv2.IMREAD_GRAYSCALE)

if img is None:
    # fallback: create a synthetic "3"-like shape for demo
    img = np.zeros((28, 28), np.uint8)
    cv2.putText(img, "3", (4, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 255, 2)

# Normalize to [0,1]
img = img.astype(np.float32) / 255.0

# Reshape to NCHW = (1,1,28,28)
img = img.reshape(1, 1, 28, 28)

# --- Run inference ---
input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name

pred = session.run([output_name], {input_name: img})[0]
digit = int(np.argmax(pred))

print(f"Predicted digit: {digit}")



```

---

## tutorials

- [Unlocking the Power of ONNX: A Beginner’s Guide with Practical Examples by using 80–20 rule](https://medium.com/@syedhamzatahir1001/unlocking-the-power-of-onnx-a-beginners-guide-with-practical-examples-by-using-80-20-rule-bd57d8fb54c8)
- [Unlocking the Power of ONNX: A Beginner’s Guide with Practical Examples by using 80–20 rule (Part-2)](https://medium.com/@syedhamzatahir1001/unlocking-the-power-of-onnx-a-beginners-guide-with-practical-examples-by-using-80-20-rule-pt-2-b799e5398d8a)

## References

- [onnx tutorial](https://www.youtube.com/playlist?list=PLkz_y24mlSJZJx9sQJCyFZt50S4ji1PeR)
- [ONNX Explained with Example | Quick ML Tutorial](https://www.youtube.com/watch?v=cZtXdMao7Ic)
