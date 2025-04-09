# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set non-interactive mode for apt-get to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update package list and install X11 and OpenGL dependencies
RUN apt-get update && apt-get install -y \
    # Core X11 libraries
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxinerama1 \
    libxcursor1 \
    libxrandr2 \
    libxi6 \
    # OpenGL/Mesa for rendering
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    # Utilities for X11 auth and testing
    xauth \
    mesa-utils \
    # apps
    cheese \
    gstreamer1.0-alsa \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables for X11
ENV DISPLAY=:0
# Optional: Define a default runtime directory (can be overridden at runtime)
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Create a non-root user (optional, for better security and matching host permissions)
ARG USERNAME=user
ARG UID=1000
ARG GID=1000

# add new sudo user
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        usermod -aG video $USERNAME && \
        usermod -aG dialout $USERNAME && \
        mkdir /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $GID $USERNAME

# Switch to the non-root user
# USER $USERNAME

# Set working directory
WORKDIR /home/$USERNAME

# Default command (can be overridden)
CMD ["bash"]