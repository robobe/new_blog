version="4.10.0"
folder="opencv"
# mkdir $folder
cd ${folder}
# curl -L https://github.com/opencv/opencv/archive/${version}.zip -o opencv-${version}.zip
# curl -L https://github.com/opencv/opencv_contrib/archive/${version}.zip -o opencv_contrib-${version}.zip
# unzip opencv-${version}.zip
# unzip opencv_contrib-${version}.zip
# rm opencv-${version}.zip opencv_contrib-${version}.zip
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

make -j8