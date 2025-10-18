---
title: Using onnx inference using C++
tags:
    - onnx
    - cpp
---


{{ page_folder_links() }}

## Install

- Download from  [releases page](https://github.com/microsoft/onnxruntime/releases)
- Extract and copy to include and lib folders

```bash
# version 1.23
tar -xzf onnxruntime-linux-x64-gpu-1.23.1.tgz
sudo cp -r onnxruntime-linux-x64-gpu-1.23.1/include /usr/local/include/onnxruntime
sudo mkdir /usr/local/lib/onnxruntime/
sudo cp -r onnxruntime-linux-x64-gpu-1.23.1/lib/* /usr/local/lib/onnxruntime/
sudo ldconfig
```

## Demo

```cpp
#include <iostream>
#include <onnxruntime_cxx_api.h>

int main() {
    // Initialize ONNX Runtime environment
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "providers_info");

    // Print version info
    std::cout << "ONNX Runtime version: " << Ort::GetVersionString() << std::endl;

    // Get available execution providers
    auto providers = Ort::GetAvailableProviders();

    std::cout << "Available providers:" << std::endl;
    for (const auto& p : providers) {
        std::cout << "  - " << p << std::endl;
    }

    // Device info — equivalent to Python's ort.get_device()
    // (C++ API doesn't expose a global 'get_device', so we check provider priority)
    if (std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") != providers.end()) {
        std::cout << "Default device: GPU (CUDA)" << std::endl;
    } else {
        std::cout << "Default device: CPU" << std::endl;
    }

    return 0;
}
```

```c
cmake_minimum_required(VERSION 3.10)
project(onnxruntime_info)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Point to ONNXRuntime headers & libs that install from tar
include_directories(/usr/local/include/onnxruntime)
link_directories(/usr/local/lib/onnxruntime)

add_executable(onnx_info src/check.cpp)
target_link_libraries(onnx_info onnxruntime)
```

```bash title="output"
ONNX Runtime version: 1.23.1
Available providers:
  - TensorrtExecutionProvider
  - CUDAExecutionProvider
  - CPUExecutionProvider
Default device: GPU (CUDA)
```

---

## Reference
- [Number recognition with MNIST in C++ ](https://onnxruntime.ai/docs/tutorials/mnist_cpp.html)