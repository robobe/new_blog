---
title: Launch process
tags:
    - ros
    - launch
    - process
---
{{ page_folder_links() }}

Launch Process from launch file

```python

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


PKG = 'ardupilot_bringup'

def generate_launch_description():
    package_share_dir = get_package_share_directory(PKG)
    sitl_bin = f"{package_share_dir}/bin/arducopter"
    param_path = f"{package_share_dir}/params/copter_default.param"

    ld = LaunchDescription()
    sitl = ExecuteProcess(
            cmd=[sitl_bin, '--model', 'quad', '--speedup', '1', '--slave', '0',
                 '--defaults', param_path,
                 '--sim-address', '127.0.0.1', '-I0'],
            output='screen'
        )
    
    ld.add_action(sitl)
    return ld
```

