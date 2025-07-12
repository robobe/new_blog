---
tags:
    - docker
    - tips
---

# Docker tips

## images

### Remove all dangling images

```bash
docker image prune
```

---

### Prompt

```bash
PS1='🐳  \[\033[1;36m\]\h \[\033[1;34m\]\W\[\033[0;35m\] \[\033[1;36m\]# \[\033[0m\]'
```

![alt text](images/docker_prompt.png)

---


### Devices

#### Demo: share pixhawk device with docker


```bash title="host"
dmesg

usb 3-4.1.5.4: Product: CubeOrange
usb 3-4.1.5.4: Manufacturer: Hex/ProfiCNC
usb 3-4.1.5.4: SerialNumber: 1F0048001451303039333335
cdc_acm 3-4.1.5.4:1.0: ttyACM0: USB ACM device
```

```bash title="host"
ll /dev/ttyACM0
crw-rw---- 1 root dialout 166, 0 Jul 11 07:19 /dev/ttyACM0
```

```yaml title="add to compose"
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
    group_add:
      - dialout
    device_cgroup_rules:
      - 'c 166:* rmw'
```

#### Demo: share by name
```bash
ll /dev/serial/by-id/
#
lrwxrwxrwx 1 root root 13 Jul 12 12:12 usb-ArduPilot_fmuv2_430032000551353532333634-if00 -> ../../ttyACM0

```

```yaml title="compose"
devices:
    - /dev/serial/by-id/usb-ArduPilot_fmuv2_430032000551353532333634-if00:/dev/ttyPIXHawk
  group_add:
    - dialout
  device_cgroup_rules:
    - 'c 166:* rmw'
  restart:
```