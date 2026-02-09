---
tags:
    - ros
    - launch
    - actions
    - node
---
{{ page_folder_links() }}
# ROS2 Launch Actions

- Node (from launch_ros.actions): Launches a ROS 2 node.
- IncludeLaunchDescription: Embeds another launch file.
- ExecuteProcess: Runs a generic process.
- [DeclareLaunchArgument](/ROS/ros_eco/launch/python/launch_argument): Defines launch arguments.
    
## Node


- package: The ROS 2 package containing the node.
- executable: The name of the executable (e.g., script or binary).
- name: The name of the node instance (optional, overrides the default node name).
- parameters: A list or dict of parameters to pass to the node.
- arguments: Command-line arguments for the executable (non-ROS args).
- output='screen': Ensures logs are visible in the terminal
- respawn=True: Automatically restarts the node if it crashes.
- respawn_delay=5.0: Delay before respawning the node.
- namespace: Places the node in a ROS 2 namespace, prefixing its topics, services, and parameters.
- remappings: Renames topics or services the  node uses. list of tuples. `[('old_topic', 'new_topic')]`
- additional_env: Sets environment variables for the node. Dictionary of key-value pairs.
- condition: Specifies a condition for launching the node. If the condition is not met, the node won't be launched.
- prefix: Adds a command prefix (e.g., gdb for debugging).
- emulate_tty: ??