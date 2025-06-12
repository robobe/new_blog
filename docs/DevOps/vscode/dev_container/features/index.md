---
tags:
    - vscode
    - devcontainers
    - features
---

# Dev container features

Dev container features are modular, pre-built scripts and configurations that automatically install tools, libraries, or settings inside your dev container, without needing to write a full Dockerfile.

They are reusable building blocks to build and config containers.

!!! note 
    The feature run after The docker finish it build stage


Check feature templates [more](https://github.com/devcontainers/feature-starter)

## Demo: Add none root user using feature

```
├── .devcontainer
│   ├── devcontainer.json
│   ├── Dockerfile
│   └── features
│       └── none_root_user
│           ├── devcontainer-feature.json
│           └── install.sh
├── docker-compose.yaml
```

```json title=".devcontainer.json"
{
    "name": "demo",
    "dockerComposeFile": [
      "../docker-compose.yaml",
    ],
    "features": {
      "./features/none_root_user": {
        "user": "ros"
      }
    },
    "service": "dev",
    "shutdownAction": "stopCompose",
    "workspaceFolder": "/workspace",
    "customizations": {
      "vscode": {
        "extensions": [
        ],
        "settings": {}
      }
    }
  }
```

```json title=".devcontainer/features/none_root_user/devcontainer-feature.json"
{
    "name": "add none root user",
    "id": "hello",
    "version": "1.0.0",
    "description": "add a non-root user with sudo",
    "options": {
        "user": {
            "type": "string",
            "proposals": [
                "user"
            ],
            "default": "user",
            "description": "The name of the user to create."
        }
    }
}
```

```bash title=".devcontainer/features/none_root_user/install.sh"
#!/bin/bash
set -e


USERNAME=${USER}
USER_UID=1000
USER_GID=$USER_UID
# Check if "ubuntu" user exists, delete it if it does, then create the desired user
if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"
# Add sudo support for the non-root user
apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*
```

!!! note "variable"
    To use variable declare in `devcontainer-feature.json` in the install.sh use the name uppercase `${USER}`
     