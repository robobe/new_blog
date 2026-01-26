---
title: Debugging plugin using vscode
tags:
    - gazebo
    - debugging
    - custom
    - plugin
---

## Docker and ptrace

!!! tip "GDB"
    don't forget to install `gdb`

```json title="devcontainer.json"
{
  "runArgs": [
    "--cap-add=SYS_PTRACE",
    "--security-opt=seccomp=unconfined"
  ]
}

```

OR

```yaml title="docker-compose"
services:
  gazebo:
    build: 
      dockerfile: .devcontainer/Dockerfile
      context: .
    cap_add:
      - SYS_PTRACE
    security_opt:
      - seccomp=unconfined
```


## VScode

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "(gdb) Launch",
      "type": "cppdbg",
      "request": "attach",
      "program": "/usr/bin/ruby3.0",
      "processId": "${command:pickProcess}",
      "MIMode": "gdb",
      "miDebuggerPath": "/usr/bin/gdb",
    }
  ]
}
```

## TODO: explain how i get `/usr/bin/ruby3.0`




---

## Reference
- [Debugging ROS2 Gazebo Plugins With VSCode](https://medium.com/@arshad.mehmood/debugging-ros2-gazebo-plugins-with-vscode-14de44d58cc9)