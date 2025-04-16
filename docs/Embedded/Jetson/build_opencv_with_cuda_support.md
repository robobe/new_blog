---
tags:
    - nvidia
    - jetson
    - opencv
    - cuda
---
# OpenCV with cuda support on jetson
Build OpenCV with CUDA support on Jetson 
Pack it using DEB for future use.


Build scripts from [ Qengineering/Install-OpenCV-Jetson-Nano
](https://github.com/Qengineering/Install-OpenCV-Jetson-Nano)

```bash
>>> import cv2
>>> cv2.__version__
'4.10.0'
>>> cv2.cuda.getCudaEnabledDeviceCount()
1
```

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
-D WITH_OPENCL=OFF \
-D CUDA_ARCH_BIN=${ARCH} \
-D CUDA_ARCH_PTX=${PTX} \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D WITH_CUBLAS=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_FAST_MATH=ON \
-D OPENCV_DNN_CUDA=ON \
-D ENABLE_NEON=ON \
-D WITH_QT=OFF \
-D WITH_OPENMP=ON \
-D BUILD_TIFF=ON \
-D WITH_FFMPEG=ON \
-D WITH_GSTREAMER=ON \
-D WITH_TBB=ON \
-D BUILD_TBB=ON \
-D BUILD_TESTS=OFF \
-D WITH_EIGEN=ON \
-D WITH_V4L=ON \
-D WITH_LIBV4L=ON \
-D WITH_PROTOBUF=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_EXAMPLES=OFF \
-D CMAKE_CXX_FLAGS="-march=native -mtune=native" \
-D CMAKE_C_FLAGS="-march=native -mtune=native" \
-D CPACK_BINARY_DEB=ON \
-D CPACK_PACKAGE_VERSION=4.10.0 \
-D CPACK_DEBIAN_PACKAGE_VERSION=4.10.0-1 \
..
```

!!! note "CUDA_ARCH_BIN, CUDA_ARCH_PTX"
     
!!! note "DEB"
    The CPACK_BINARY_DEB option in OpenCV’s CMake configuration enables packaging OpenCV as a Debian (.deb) package using CPack.

    ```
    -D CPACK_BINARY_DEB=ON
    -D CPACK_PACKAGE_VERSION=4.10.0 
    -D CPACK_DEBIAN_PACKAGE_VERSION=4.10.0-1
    ```

    ```
    make package ..
    ```

     
### Check installation

```bash title="pull cuda runtime image"
docker pull nvidia/cuda:12.6.0-runtime-ubuntu22.04
```

```bash title="run and share cv build folder"
# from build folder
docker run  --gpus all --runtime=nvidia \
-it --rm --hostname test \
-v `pwd`:/tmp/cv \
nvidia/cuda:12.6.0-runtime-ubuntu22.04 /bin/bash

```

```bash title="install dependencies"
apt update
apt install cudnn9
apt install python3
apt install python3-numpy
#
#install all opencv debs
cd /tmp/cv
apt -i *.deb
```

```bash title="check cuda installation"
>>> import cv2
>>> cv2.__version__
'4.10.0'
>>> cv2.cuda.getCudaEnabledDeviceCount()
1
```