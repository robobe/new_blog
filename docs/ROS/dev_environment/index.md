# ROS2 Dev Build Test Environment

<div class="grid-container">
    <div class="grid-item">
        <a href="dev">
        <img src="images/ros_dev.png" width="150" height="150">
        <p>Development</p>
        </a>
    </div>
    <div class="grid-item">
    <a href="build">
        <img src="images/ros_dev.png" width="150" height="150">
        <p>Build</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="prod">
        <img src="images/ros_dev.png" width="150" height="150">
        <p>Prod</p>
        </a>
    </div>
    
</div>

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