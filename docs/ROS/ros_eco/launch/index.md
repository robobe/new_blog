---
tags:
  - ros
  - launch
---
{{ page_folder_links() }}

# ROS2 launch
The ROS 2 launch system is a declarative orchestration engine.
You don’t run code — you describe a system: which processes should exist, how they’re configured, and how they relate to each other. The launch system then executes and supervises that description, reacting to events (start, exit, failure) at runtime.


<div class="grid-container">
    <div class="grid-item">
        <a href="python">
            <img src="images/python.png" width="150" height="150">
            <p>PYTHON</p>
             </a>
        </div>
    <div class="grid-item">
        <a href="yaml">
        <img src="images/yaml.png" width="150" height="150" >
        <p>YAML</p>
        </a>
    </div>
    

</div>

## Minimal YAML

```yaml
launch:

- node:
    pkg: demo_nodes_cpp
    exec: talker

- node:
    pkg: demo_nodes_cpp
    exec: listener
```


---

## Minimal example
```python title="minimal launch file to run ros2 node"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package='your_package_name',
        executable='your_node_executable',
        name='your_node_name',
        output='screen'
    )

    ld.add_action(node)
    
    return ld
```

```cmake title="cmake copy launch to share"
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```

## ROS2 Launch system API



- **launch.substitutions**: Tools for dynamic values, e.g.:
    - LaunchConfiguration: References launch arguments.
    - TextSubstitution: Inserts static text.
    - PythonExpression: Evaluates Python expressions.
- **launch.conditions**: Conditional execution (e.g., IfCondition, UnlessCondition).

- [DeclareLaunchArgument and LaunchConfiguration](launch_config_and_argument.md)
- [PathJoinSubstitution](launch_path_and_paramfile.md)
- [IncludeLaunchDescription](launch_include_demo.md)
- [EnvironmentVariable](environment_variable.md)
- [Conditions]()
- [events]()