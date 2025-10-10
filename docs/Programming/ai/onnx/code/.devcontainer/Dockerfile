FROM nvidia/cuda:12.6.0-cudnn-runtime-ubuntu22.04

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Install Python and pip
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
        python3-pip \
        python3-dev \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Copy and install requirements
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# RUN pip3 install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
RUN pip3 install https://pypi.jetson-ai-lab.io/jp6/cu126/+f/e1e/9e3dc2f4d5551/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl#sha256=e1e9e3dc2f4d5551c5f0b5554cf490d141cd0d339a5a7f4826ac0e04f20a35fc

# RUN apt-get update \
#     && apt-get -y install --no-install-recommends \
#         wget \
#         libgl1 \
#         libglib2.0-0 \
#         libsm6 \
#         libxext6 \
#         libxrender-dev \
#     && apt-get clean -y \
#     && rm -rf /var/lib/apt/lists/*