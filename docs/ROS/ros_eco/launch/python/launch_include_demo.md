---
tags:
    - ros2
    - launch
    - python
    - IncludeLaunchDescription
    - PythonLaunchDescriptionSource
---

# IncludeLaunchDescription

IncludeLaunchDescription method allows to include one launch description (essentially a sub-launch file) inside another, enabling better organization and scalability of complex robot systems. This is particularly useful when you want to break down a large system into smaller, reusable components.


## Demo: Very simple
Parent launch file `include_demo.launch.py` include the `args_log_demo.launch.py` as sub launch

- `args_log_demo.launch.py` use as sub launch
- `include_demo.launch.py` us as parent launch

```python title="args_log_demo.launch.py"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()
    
    arg1_decalre = DeclareLaunchArgument('arg1', description="simple arg1 for demo", default_value='hello world')
    arg1 = LaunchConfiguration('arg1')

    log1 = LogInfo(msg=["Argument 'arg1' value: ",arg1])
   
    ld.add_action(arg1_decalre)
    ld.add_action(log1)
    return ld

```

```python title="include_demo.launch.py"
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    my_robot_pkg = get_package_share_directory('launch_tutorial')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_pkg, 'launch', 'args_log_demo.launch.py')
        ),
        launch_arguments={
            'arg1': 'call from parent',
        }.items()
    )

    ld.add_action(robot_launch)
    return ld
```

### usage

```bash
ros2 launch launch_tutorial include_demo.launch.py 
# result
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Argument 'arg1' value: call from parent
```