---
tags:
    - opencv
---


<div class="grid-container">
    <div class="grid-item">
        <a href="build">
            <img src="images/build.png"  width="150" height="150">
            <p>Build</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="cuda">
            <img src="images/cuda.png"  width="150" height="150">
            <p>CUDA</p>
        </a>
    </div>
    <div class="grid-item">
        <a href=optical_flow>
            <img src="images/of.png"   width="150" height="150">
            <p>Optical Flow</p>
            </a>
        </div>

</div>

---

## CPP

```bash title="install using apt"
# install headers, shared library and cmake config files
sudo apt install libopencv-dev
```

### Demo

```cpp
--8<-- "code/hello_cv.cpp"
```

```cpp
--8<-- "code/CMakeLists.txt"
```