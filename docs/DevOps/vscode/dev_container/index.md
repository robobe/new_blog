---
tags:
    - vscode
    - devcontainer
    - docker
---


# VSCode Dev Container

<div class="grid-container">
    <div class="grid-item">
        <a href="features">
            <p>features</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="">
            <p>TBD</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="">
            <p>TBD</p>
        </a>
    </div>

</div>


```yaml
{
    "name": "<name>",
    "dockerFile": "Dockerfile",
    "context": "..",
    "workspaceFolder": "/workspaces/<host folder name>",
    "remoteUser": "user",
    "runArgs": [
        "--net=host",
        "--hostname=<name>",
        "--add-host=<name>:127.0.0.1"
    ],
    "remoteEnv": {
        "DISPLAY": "${localEnv:DISPLAY}",
        "QT_X11_NO_MITSHM": "1"
    },
    "mounts": [
        "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind",
        "source=/dev/dri,target=/dev/dri,type=bind"
    ],
    "postCreateCommand": "/home/user/.venv/bin/pip install -r /workspaces/${containerWorkspaceFolderBasename}/requirements.txt",
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-python.python",
                "ms-python.flake8",
                "ms-python.black-formatter",
                "ms-toolsai.jupyter"
            ],
            "settings": {}
        }
    }
}
```

!!! Tip dri
    Mounting /dev/dri between the host and the container is needed only when you want hardware-accelerated rendering

---

## Resource
- [Development Containers ](https://containers.dev/): An open specification for enriching containers with development specific content and settings. 
- [Run DevContainer on remote host](vscode_devcontainer_remote_host.md)
- [VScode docker volume](vscode_docker_volume.md)