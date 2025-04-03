---
tags:
    - ros2
    - launch
    - python
    - DeclareLaunchArgument
    - LaunchConfiguration
---

# DeclareLaunchArgument and LaunchConfiguration

## DeclareLaunchArgument
Defines an argument (a variable) that can be passed to a launch file via  **CLI** , each argument can have default value.


## LaunchConfiguration
LaunchConfiguration represents the value of a launch argument at **runtime**. It acts as a placeholder or reference to an argument declared with DeclareLaunchArgument


```python title="launch_argument_demo.launch.py"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()
    
    arg1_decalre = DeclareLaunchArgument('arg1', description="simple arg1 for demo", default_value='hello world')
    arg1 = LaunchConfiguration('arg1')

    log1 = LogInfo(msg=["argument demo from cli using subsitution ",arg1])
   
    ld.add_action(arg1_decalre)
    ld.add_action(log1)
    return ld
```

## usage

```bash
# Output default value
ros2 launch launch_tutorial args_log_demo.launch.py 
#
[INFO] [launch.user]: Argument 'arg1' value: hello world

# Show launch arguments
ros2 launch launch_tutorial args_log_demo.launch.py -s
#
Arguments (pass arguments as '<name>:=<value>'):

    'arg1':
        simple arg1 for demo
        (default: 'hello world')

# Get Argument from CLI
ros2 launch launch_tutorial args_log_demo.launch.py arg1:="data to arg1 from cli"
#
[INFO] [launch.user]: Argument 'arg1' value: data to arg1 from cli
```