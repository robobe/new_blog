---
tags:
    - vscode
    - cross compiler
---

{{ page_folder_links() }}

## Using vscode with cross compiler

### Install
```bash
sudo apt install gcc-aarch64-linux-gnu
```

### Build 
#### current file
```json title="tasks.json"
{
    "label": "build-arm",
    "type": "shell",
    "command": "aarch64-linux-gnu-g++",
    "args": [
        "-g",
        "-o",
        "${workspaceFolder}/build/${fileBasenameNoExtension}_arm",
        "${file}"
    ],
    "group": {
        "kind": "build",
        "isDefault": true
    },
    "presentation": {
        "echo": true,
        "reveal": "always",
        "focus": false,
        "panel": "shared"
    },
    "problemMatcher": ["$gcc"]
}
```

### c_cpp_properties
```json title="c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "ARM Cross Compile",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/aarch64-linux-gnu//include/**"
            ],
            "compilerPath": "/usr/bin/aarch64-linux-gnu-gcc",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-arm"
        }
    ],
    "version": 4
}
```