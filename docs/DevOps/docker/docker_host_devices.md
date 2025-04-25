---
tags:
    - docker
    - devices
    - hardware
    - usb
    - joystick
---

# Access Host device in docker
Connect and share hardware device with docker container



## joystick

### host
```bash
# check dmesg

# check device
ll /dev/input/js0

crw-rw-r--+ 1 root input 13, 0 Apr 25 07:44 /dev/input/js0
```

```bash title="test joy functionality"
sudo apt install joystick

## test
jstest /dev/input/js0
```

### docker

```bash title="share device with container"
docker run -it --rm \
--name test \
--hostname test \
--device /dev/input/js0 \
ubuntu:22.04 \
/bin/bash
```

#### device-cgroup-rule
Docker option used to grant fine-grained access to devices from the Linux cgroup (control group) level.

!!! note "cgroup rules"
     Linux cgroups (control groups) are a powerful kernel feature that allows you to limit, isolate, and prioritize resources (like CPU, memory, devices, I/O, etc.) for groups of processes. [more](https://man7.org/linux/man-pages/man7/cgroups.7.htmldo)
     

```bash
--device-cgroup-rule="type major:minor permissions"
```

!!! note "permission"
    - r(read)
    - w(write)
    - m(mknod- permission to create spacial device file)

```bash title="joystick check type and major, minor"
ll /dev/input/js0

crw-rw-r--+ 1 root input 13, 0 Apr 25 07:44 /dev/input/js0
```

- type: `c`
- major: 13
- minor: 0

```bash title="run docker with device cgroup rule"
docker run -it --rm \
--name test \
--hostname test \
-v /dev/input:/dev/input \
--device-cgroup-rule='c 13:* rwm' \
ubuntu:22.04 \
/bin/bash
```

!!! tip "privileged"
    **No** `--privileged` mode need to run the container and share the device
     

### Demo
- Run docker with none root permission
- Share joystick
- Connect the joystick after the container run
- Disconnect and connect again


```dockerfile title="Dockerfile.joytest"
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends \
        joystick \
    && rm -rf /var/lib/apt/lists/*



RUN groupadd -g 107 input

ARG USERNAME=user
ARG UID=1000
ARG GID=1000


# add new sudo user
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    usermod -aG input $USERNAME && \
    usermod -aG dialout $USERNAME && \
    mkdir /etc/sudoers.d && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod  --uid $UID $USERNAME && \
    groupmod --gid $GID $USERNAME
        
CMD ["/bin/bash"]
```

!!! note "input group"
    Create `input` group on docker 

    ```
    RUN groupadd -g 107 input
    ```

    Add user (none root) to `input group`

    ```
    usermod -aG input $USERNAME && \
    ```
     

```bash title="build"
docker build -f Dockerfile.joytest -t joystick:test .
```

```bash title="run and test"
docker run -it --rm \
--user user \
--name test \
--hostname test \
-v /dev/input:/dev/input \
--device-cgroup-rule='c 13:* rwm' \
joystick:test \
/bin/bash
```

```bash title="test - joystick"
ll /dev/input/js0
jstest /dev/input/js0
```

#### run all the Scenarios
- Run container with joy connect. disconnect and connect again
- Run container when joy disconnect, connect and check


