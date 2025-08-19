---
title: NVidia VPI
tags:
    - vpi
    - nvidia
---

{{ page_folder_links() }}

NVIDIA® Vision Programming Interface (VPI) is a software library that implements computer vision (CV) and image processing (IP) algorithms on several computing hardware platforms available in NVIDIA embedded and discrete devices.
VPI provides seamless access to computer hardware. 
Within VPI, the same algorithm is implemented in different backends, such as CPU, GPU, PVA1, VIC2 and OFA
[READ More](https://docs.nvidia.com/vpi/index.html)

!!! note "nvidia"
    PVA - Programmable Vision Accelerator 
    VIC - Video Image Compositor 
    OFA -  


     
---

## install
[nvidia documents](https://docs.nvidia.com/vpi/installation.html)

!!! note "vpi installation location"

    ```bash
    /opt/nvidia/vpi3
    ```

---

## Basic Concepts
- Stream
- backends
- Algorithms
- Data Buffers


### Backend
A backend comprises the compute hardware that ultimately runs an algorithm

| Backend | Device/platform                                                |
|---------|---------------------------------------------------------------|
| CPU     | All devices on x86 (linux) and Jetson aarch64 platforms       |
| CUDA	  | All devices on x86 (linux) with a Maxwell or superior NVIDIA GPU, and Jetson aarch64 platforms |
| PVA	  | All Jetson AGX Orin and Jetson Orin NX devices |
| VIC	  | All Jetson devices. |
| OFA	  | All Jetson Orin devices.  |

---

## Hello VPI
Very simple application
convert image to gray using PIL and VPI

```python
--8<-- "docs/Programming/vision/nvidia/vpi/code/hello_vpi.py"
```
     
### Check if backend available

```python
import vpi

print("CUDA available? ", bool(vpi.Backend.CUDA & vpi.Backend.AVAILABLE))
print("PVA available? ", bool(vpi.Backend.PVA & vpi.Backend.AVAILABLE))
print("VIC available? ", bool(vpi.Backend.VIC & vpi.Backend.AVAILABLE))
print("OFA available? ", bool(vpi.Backend.OFA & vpi.Backend.AVAILABLE))
```


### Convert vpi image to numpy view
     
```python title="vpi to numpy view "
import matplotlib.pyplot as plt
import vpi

# img_vpi is a vpi.Image (any backend). Ensure it's 8-bit gray.
img_u8 = img_vpi.convert(vpi.Format.U8)        # or skip if already U8

with img_u8.rlock_cpu() as np_img:             # NumPy view (no copy)
    plt.imshow(np_img, cmap='gray', vmin=0, vmax=255)
    plt.axis('off')
    plt.show()

```


---

# LK

```python
 class vpi.OpticalFlowPyrLK(frame: vpi.Image, 
    keypoints: vpi.Array, 
    levels: int, *, 
    kptstatus: vpi.Array = None, 
    scale: float = 0.5, 
    backend: vpi.Backend = vpi.Backend.DEFAULT) → vpi.OpticalFlowPyrLK
```



---

## Reference
- [Introduction to NVIDIA VPI](https://youtu.be/6-FhpfwBpXk)
- [Implementing computer vision and image processing using VPI](https://info.nvidia.com/rs/156-OFN-742/images/Webinar_PDF-Implementing_computer_vision_image_processing_solutions_with_VPI.pdf)
- [Python API](https://docs.nvidia.com/vpi/python/index.html)