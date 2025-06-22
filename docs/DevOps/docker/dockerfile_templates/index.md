---
tags:
    - docker
    - dockerfile
    - templates
    - user
    - pip
---

# Dockerfile template and snippets
Dockerfile templates and snippets for various applications and services.

<div class="grid-container">
    <div class="grid-item">
        <a href="#user">
           <p>Non root user</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#gstreamer">
           <p>GStreamer snippet</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#architecture-condition">
           <p>Architecture condition</p>
        </a>
    </div>
</div>

## User

Add user and install sudo support

```dockerfile title="Dockerfile"
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

```

### usage

```bash title="Terminal"
docker run --rm -it --user user my_ubuntu:22.04 id
#
uid=1000(user) gid=1000(user) groups=1000(user)
```

---

## User with device access
#TODO: move to embedded section
```dockerfile
```


### usage

```bash
docker run --rm -it \
--user user \
--group-add=dialout \
--device /dev/ttyACM0 \
my_ubuntu:22.04 \
ll /dev/ttyACM0
```

---

## Architecture condition

```
RUN ARCH=$(uname -m) && \
    apt update && \
    if [ "$ARCH" = "aarch64" ]; then \
        apt install -y ./OpenCV-unknown-aarch64-*.deb; \
    elif [ "$ARCH" = "x86_64" ]; then \
        apt install -y ./OpenCV-unknown-x86_64-*.deb; \
    else \
        echo "Unsupported architecture: $ARCH"; exit 1; \
    fi && \
```

---

## GStreamer

```dockerfile
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
        libgstreamer1.0-0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-tools \
        python3-gi \
        gir1.2-gstreamer-1.0 \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-libav \
        gstreamer-1.0 \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
```