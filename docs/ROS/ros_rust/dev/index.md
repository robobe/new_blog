---
title: ROS2 Rust VSCode environment
tags:
    - ros
    - rust
    - vscode
    - devcontainer
    - docker
---


## Docker
- Clone [ros2_rust](https://github.com/ros2-rust/ros2_rust/tree/main) to workspace src folder
    - this ws use as docker builder
- Build docker [check ros2_rust building.md](https://github.com/ros2-rust/ros2_rust/blob/main/docs/building.md#option-2-using-the-docker-image)
- Use the above docker as base
    - Add user
    - Install rust in user scope
- clone all package from `ros2_rust_humble.repos`

### Add .devcontainer

<details>
<summary>devcontainer.json</summary>

```json
{
    "name": "ros2_rust",
    "dockerComposeFile": "../docker-compose.yaml",
    "service": "ros",
    "shutdownAction": "stopCompose",
    "workspaceFolder": "/workspace",
    "customizations": {
      "vscode": {
        "extensions": [
            "ms-python.python",
            "ms-vscode.cpptools",
            "twxs.cmake",
            "redhat.vscode-xml",
            "redhat.vscode-yaml",
            "albert.tabout",
            "actboy168.tasks",
            "streetsidesoftware.code-spell-checker",
            "mhutchie.git-graph",
            "dlech.chmod",
            "smilerobotics.urdf"
        ],
        "settings": {}
      }
    }
  }
```

</details>


<details>
<summary>docker-compose.yaml</summary>

```yaml
services:
  ros:
    build: 
      context: .
      dockerfile: .devcontainer/Dockerfile
    user: "user"
    volumes:
      - .:/workspace:cached
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # - .gazebo:/home/user/.gazebo:cached 
      - /dev/dri:/dev/dri            # keep for OpenGL context
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia        # explicitly use NVIDIA runtime
              count: all
              capabilities: [gpu]
    hostname: ros
    extra_hosts:
      - "ros:127.0.0.1"
    network_mode: host
    stdin_open: true
    tty: true
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      - __NV_PRIME_RENDER_OFFLOAD=1 # Even though NVIDIA is not the primary GPU, allow this process to render on the NVIDIA GPU.”
      - __GLX_VENDOR_LIBRARY_NAME=nvidia # “Use the NVIDIA GLX implementation (libGLX_nvidia.so) instead of Mesa.”
      - XDG_RUNTIME_DIR=/tmp/runtime-user
    runtime: nvidia
```

</details>

<details>
<summary>Dockerfile</summary>

```dockerfile
FROM ros2_rust_dev:latest

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

# Set up autocompletion for user
RUN apt-get update && apt-get install -y --no-install-recommends git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*


# Install Rust
USER user
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.85.0 -y
ENV PATH=/home/user/.cargo/bin:$PATH
```
</details>


```bash
vcs import src < src/ros2_rust/ros2_rust_humble.repos
```

!!! warning ""
    I add `COLCON_IGNORE` file to `parameter_demo` folder under `ros2-rust/examples/rclrc`

```bash title="build"
source /opt/ros/humble/setup.bash
colon build
```

```bash title="run example"
```

---

## Reference
- [Programming ROS 2 with RUST](https://www.youtube.com/watch?v=yButNTZupF8)