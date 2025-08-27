---
title: Nvidia nsight system
tags:
    - nvidia
    - profile
    - nsight
    - nsys
---

{{ page_folder_links() }}



[Download NVIDIA Nsight Systems](https://developer.nvidia.com/nsight-systems/get-started)

---
Nvidia profiling tools


## nsys

```bash
sudo apt install nvidia-nsight-systems
```

#TODO: check
```bash 
nsys profile -o output python3 your_vpi_script.py


```

```python
#!/usr/bin/env python3
import vpi
import numpy as np
import time

# Maximum performance on Jetson
import subprocess
subprocess.run("sudo jetson_clocks", shell=True)

img = np.random.randint(0, 255, (1080, 1920, 3), dtype=np.uint8)

start = time.time()
with vpi.Backend.CUDA:
    frame = vpi.asimage(img, vpi.Format.BGR8).convert(vpi.Format.U8)
    corners, scores = frame.harriscorners()
    
# This causes GPU→CPU transfer (shows in profiler)
corners_cpu = corners.cpu()
print(f"Time: {(time.time()-start)*1000:.2f}ms")
```

```bash
nsys profile -t cuda --stats=true python3 test.py

```

- [CUDA memcpy HtoD] = CPU→GPU transfers (minimize these)
- [CUDA memcpy DtoH] = GPU→CPU transfers (minimize these)
- [CUDA kernel] = VPI operations on GPU