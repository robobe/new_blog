---
tags:
    - ros
    - vscode
    - settings
    - extensions
---
# VSCode settings and extension for ROS

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