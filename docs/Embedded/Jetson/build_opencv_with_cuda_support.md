---
tags:
    - nvidia
    - jetson
    - opencv
    - cuda
---

Build scripts from [ Qengineering/Install-OpenCV-Jetson-Nano
](https://github.com/Qengineering/Install-OpenCV-Jetson-Nano)

```
>>> import cv2
>>> cv2.__version__
'4.10.0'
>>> cv2.cuda.getCudaEnabledDeviceCount()
1
>>> 
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
-D CPACK_BINARY_DEB=ON  ..
```

!!! note "DEB"
    The CPACK_BINARY_DEB option in OpenCV’s CMake configuration enables packaging OpenCV as a Debian (.deb) package using CPack.

    `-D CPACK_BINARY_DEB=ON`

    ```
    make package ..
    ```

     