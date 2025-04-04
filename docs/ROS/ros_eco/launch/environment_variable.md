---
tags:
    - ros2
    - launch
    - python
    - EnvironmentVariable
---


```python title="set use_sim_time parameter from environment variable"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PythonExpression

def generate_launch_description():
    # Get the environment variable as a string
    use_sim_time_str = EnvironmentVariable('USE_SIM_TIME', default_value='false')

    # Convert to boolean using PythonExpression
    use_sim_time = PythonExpression(["'", use_sim_time_str, "' == 'true'"])

    example_node = Node(
        package='your_package_name',
        executable='your_node_executable',
        name='example_node',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        example_node
    ])
```