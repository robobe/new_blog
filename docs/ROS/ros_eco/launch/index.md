---
tags:
  - ros
  - launch
---

# ROS launch


```python
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

```cmake
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
```