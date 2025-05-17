ARG CUDA="12.6.0"
ARG UBUNTU="22.04"

# docker pull nvidia/cuda:12.6.0-cudnn-devel-ubuntu22.04
FROM nvidia/cuda:${CUDA}-cudnn-devel-ubuntu${UBUNTU}

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    cmake \
    cmake-data \
    gcc \
    g++ \
    ninja-build \
    gdb \
    git \
    wget \
    curl \
    unzip \
    yasm \
    doxygen \
    pkg-config \
    checkinstall \
    build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev \
    python3-dev python3-numpy \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libswscale-dev libgtk-3-dev libv4l-dev v4l-utils\
    libopenblas-dev libopenblas-base libatlas-base-dev liblapacke-dev \
    protobuf-compiler python3-dev python3-venv python3-numpy python3-wheel python3-setuptools \
&& rm -rf /var/lib/apt/lists/*
     
  

