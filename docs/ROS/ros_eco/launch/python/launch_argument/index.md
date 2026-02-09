---
title: DeclareLaunchArgument and LaunchConfiguration
tags:
    - ros2
    - launch
    - python
    - DeclareLaunchArgument
    - LaunchConfiguration
---

**DeclareLaunchArgument** defines an argument,  
**LaunchConfiguration** reads its value later (substitution).

They do different jobs and live at different phases of the launch lifecycle.

ROS 2 launch as two phases

1️⃣ Declaration phase (setup / contract)

“These are the inputs this launch file accepts”

This is where DeclareLaunchArgument lives.

2️⃣ Execution phase (runtime / resolution)

“What value does this argument have right now?”

This is where LaunchConfiguration lives.


---

## DeclareLaunchArgument
Defines an argument (a variable) that can be passed to a launch file via  **CLI** , each argument can have default value.

```python
DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time'
)
```

- Registers **use_sim_time** as a valid launch argument
- Sets a default value
- Makes it visible to:
    - ros2 launch ... --show-args
    - CLI validation

Other launch files that include this one

---

## LaunchConfiguration
LaunchConfiguration represents the value of a launch argument at **runtime**. It acts as a **placeholder** or reference to an argument declared with DeclareLaunchArgument

```python
LaunchConfiguration('use_sim_time')
```

- Creates a placeholder

Says:

“At runtime, fetch the value of **use_sim_time** from the launch context”
This is why it’s called a substitution.

---

### Demo

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

```bash title="list all launch arguments"
# Output default value
ros2 launch launch_tutorial args_log_demo.launch.py 
#
[INFO] [launch.user]: Argument 'arg1' value: hello world
```

```bash title="run with argument"
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