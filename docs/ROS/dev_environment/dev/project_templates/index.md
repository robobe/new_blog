---
tags:
    - ros
    - vscode
    - docker
    - devcontainer
    - docker compose
    - projects
---

# Ros2 projects templates

<div class="grid-container">
    <div class="grid-item">
        <a href="multi_containers">
        <p>multi container</p>
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
        </img>
    </div>
    
</div>

## ROS2 jezzy
```
├── .devcontainer
│   ├── devcontainer.json
│   ├── Dockerfile.jazzy
│   ├── Dockerfile.runtime
│   ├── Dockerfile.dev
│   └── Dockerfile
├── .vscode
│   ├── tasks.json
│   └── settings.json
├── docker-compose.yml
├── .gitignore
├── README.md
├── colcon_defaults.md
├── env.sh
└── src
```

<details>
    <summary>.devcontainer/Dockerfile.jazzy</summary>

```json
--8<-- "docs/ROS/dev_environment/dev/project_templates/jazzy/code/Dockerfile.jazzy"
```
</details>

<details>
    <summary>.devcontainer/devcontainer.json</summary>

```json
--8<-- "docs/ROS/dev_environment/dev/project_templates/jazzy/code/devcontainer.json"
```
</details>


<details>
    <summary>docker-compose.yaml</summary>

```json
--8<-- "docs/ROS/dev_environment/dev/project_templates/jazzy/code/docker-compose.yaml"
```
</details>


### ros

```yaml title="colcon_defaults.yaml"
build:
  # Use symlink install to speed up builds
  symlink-install: true
  # Set build type (e.g., Release or Debug)
  cmake-args:
    - "-DCMAKE_BUILD_TYPE=Release"

test:
  # Run tests in parallel
  parallel-workers: 4

# Additional settings for colcon test and other commands
test-result:
  verbose: true
```

### VSCode

#### task.json

```json 
{
    "version": "2.0.0",
    "tasks": [
      {
        "label": "colcon",
        "detail": "Build all ros2 packages using colcon",
        "type": "shell",
        "command": "colcon build",
        "problemMatcher": [],
        "group": {
            "kind": "build",
            "isDefault": true
        }
      }
    ]
}
```