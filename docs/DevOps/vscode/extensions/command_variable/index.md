---
tags:
    - vscode
    - extensions
    - command_variable
---

# command_variable
Extend vscode command variables to use in `tasks.json` and `launch.json`
[marketplace](https://marketplace.visualstudio.com/items?itemName=rioj7.command-variable#file-content-yaml-property)

### file pick demo
Open file select
Can filter the file list to show using include and exclude arguments
[more](https://marketplace.visualstudio.com/items?itemName=rioj7.command-variable#pick-file)

```json title="tasks.json"
{
    "version": "2.0.0",
    "tasks": [
      {
        "label": "echo select file",
        "type": "shell",
        "command": "echo",
        "args": [
          "${input:file_path}"
        ],
        "problemMatcher": []
      }
    ],
    "inputs": [
      {
        "id": "file_path",
        "type": "command",
        "command": "extension.commandvariable.file.pickFile",
        "args":{
            "include": "**/control"
        }
       
      }
    ]
}
```
