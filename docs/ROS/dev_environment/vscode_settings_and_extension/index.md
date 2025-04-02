---
tags:
    - ros
    - vscode
    - settings
    - extensions
---
# VSCode settings and extension for ROS

## VSCode extension

| ext  | language / tool  |
|---|---|
| ms-python.python  | python  |
| ms-vscode.cpptools  | cpp  |
| twxs.cmake  | cmake  |
| redhat.vscode-xml  | xml  |
| redhat.vscode-yaml  | yaml  |
| **TOOLS**  |   |
| albert.tabout  | tab out  |
| actboy168.tasks | tasks   |
| streetsidesoftware.code-spell-checker  | speller  |
| **git**  |   |
| mhutchie.git-graph  | git graph  |
| **OS**  |   |
| dlech.chmod  | set executable bit  |


```
"ms-python.python",
"ms-vscode.cpptools",
"twxs.cmake",
"redhat.vscode-xml",
"redhat.vscode-yaml",
"albert.tabout",
"actboy168.tasks",
"streetsidesoftware.code-spell-checker",
"mhutchie.git-graph",
"rioj7.command-variable",
"dlech.chmod",
"mkloubert.vscode-deploy-reloaded"
```


## VSCode settings
### Python

**python.analysis.extraPaths**: The python.analysis.extraPaths setting in Visual Studio Code is used to specify additional directories that the Python language server (Pylance) should include when analyzing your code.

**python.autoComplete.extraPaths**: The python.autoComplete.extraPaths setting in Visual Studio Code is used to specify additional directories that the Python extension should include when providing autocomplete suggestions.

```json
{
    "python.autoComplete.extraPaths": [
        "${workspaceFolder}/",
        "/opt/ros/humble/lib/python3.10/site-packages/",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"

    ],

    "python.analysis.extraPaths": [
        "${workspaceFolder}/",
        "/opt/ros/humble/lib/python3.10/site-packages/",
        "/opt/ros/humble/local/lib/python3.10/dist-packages/"
    ]
}
```

### Cpp
```json title="c_cpp_properties.json"
{
    "configurations": [
        {
            "name": "ROS 2",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/**",
                "/usr/local/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "gnu17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64",
            "browse": {
                "path": [
                    "${workspaceFolder}/**",
                    "/opt/ros/humble/include/**",
                    "/usr/include/**",
                    "/usr/local/include/**"
                ],
                "limitSymbolsToIncludedHeaders": true,
                "databaseFilename": ""
            }
        }
    ],
    "version": 4
}

```