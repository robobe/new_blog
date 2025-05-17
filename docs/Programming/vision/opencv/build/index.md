---
tags:
    - opencv
    - build
    - cuda
---

# Build opencv from source with cuda support

Using docker to build opencv from source with cuda support for python and cpp. pack the result as debs file to install on other machines.

!!! tip "Check machine CUDA version"
    ```bash
    nvidia-smi
    ```

The docker base on the host CUDA version, 
I use pre-built docker image from NVIDIA, `FROM nvidia/cuda:${CUDA}-cudnn-devel-ubuntu${UBUNTU}`,

```bash
--8<-- "docs/Programming/vision/opencv/build/code/Dockerfile"
```

### Build process

- Download and extract opencv and opencv_contrib

```bash
version="4.10.0"
folder="opencv"
mkdir $folder
cd ${folder}
curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
unzip opencv-${version}.zip
unzip opencv_contrib-${version}.zip
rm opencv-${version}.zip opencv_contrib-${version}.zip
```

- Create build folder
- Run cmake
- Build

```bash
mkdir opencv-${version}/build
cd opencv-${version}/build

cmake \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6 \
-D CUDA_ARCH_BIN="5.2 5.3 6.0 6.1 6.2 7.0 7.2 7.5 8.0 8.6" \
-D CUDA_ARCH_PTX="8.6rm -rf" \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.10.0/modules \
-D WITH_GSTREAMER=OFF \
-D WITH_LIBV4L=ON \
-D BUILD_opencv_python3=ON \
-D BUILD_TESTS=OFF \
-D BUILD_PERF_TESTS=OFF \
-D BUILD_EXAMPLES=OFF \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_QT=ON \
-D WITH_GTK=ON \
-D WITH_VTK=OFF \
-D WITH_FFMPEG=OFF \
-D WITH_1394=OFF \
-D CPACK_BINARY_DEB=ON \
-D BUILD_JAVA=OFF \
-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
-D CPACK_PACKAGE_VERSION=4.10.0 \
-D CPACK_DEBIAN_PACKAGE_VERSION=4.10.0-1 \
..

# Build
make -j8
```

### pack to debs

```bash title="pack"
make package ..
```

**Downloads**

  - [Dockerfile](docs/Programming/vision/opencv/build/code/Dockerfile)
  - [.devcontainer.json](docs/Programming/vision/opencv/build/code/devcontainer.json)
  - [build.sh](docs/Programming/vision/opencv/build/code/build.sh)

---

### Install and check

```bash title="install"
dpkg -i *.deb
```

#### check

```bash title="check cuda support"
>>> import cv2
>>> cv2.__version__
'4.10.0'
>>> cv2.cuda.getCudaEnabledDeviceCount()
1
```

```python title="simple.py"
import cv2

# Load an image from file
image = cv2.imread("/workspaces/opencv_builder/src/lena.png")

# Check if the image was loaded successfully
if image is None:
    print("Failed to load image")
else:
    # Display the image in a window
    cv2.imshow("My Image", image)

    # Wait for any key press
    cv2.waitKey(0)

    # Close all OpenCV windows
    cv2.destroyAllWindows()
```

[Download lena img](code/lena.png)


```cpp title="simple.cpp"
// g++ simple.cpp -o simple `pkg-config --cflags --libs opencv4` -I/usr/include/opencv4/

#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    // Load the image
    cv::Mat image = cv::imread("/workspaces/opencv_builder/src/lena.png");

    // Check if image loaded successfully
    if (image.empty()) {
        std::cerr << "Failed to load image" << std::endl;
        return 1;
    }

    // Show the image in a window
    cv::imshow("My Image", image);

    // Wait for a key press
    cv::waitKey(0);

    // Close all windows
    cv::destroyAllWindows();

    return 0;
}

```

```bash title="build"
g++ simple.cpp -o simple `pkg-config --cflags --libs opencv4` -I/usr/include/opencv4/
```

!!! warning "include search path"
    - `/usr/include/opencv4/` is the default include path for opencv4, if you use other version, please check the path.
    - `pkg-config --cflags --libs opencv4` will return the include path and lib path for opencv4, you can use it in your build command.
    - using `pkg-config` the include path point `-I/usr/local/include/opencv4`
    - After the installation from debs the include path is `/usr/include/opencv4/`
     