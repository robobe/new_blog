---
title: ROS yaml launch
tags:
  - ros
  - launch
  - yaml
---

{{ page_folder_links() }}

- [Minimal](#minimal)
- [let](#let)
- [Argument](#argument)
  - [basic](#basic)
  - [with default](#with-default)
  - [with multiple choice](#with-multiple-choice)
- [Rviz](#rviz)
- [Gazebo](#gazebo)

---

## Minimal

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

## let
- Create a local variable for reuse within the launch file.
- Cannot be overridden from the command line.

```yaml
launch:
- let:
    name: model_path
    value: $(find-pkg-share yaml_launch)/models
  
- executable:
    cmd: echo $(var model_path)
    output: screen
```

---

## Argument
Define an argument that can be overridden via the command line when launching

### basic
```yaml
launch:
- arg:
    name: simple_arg
  
- executable:
    cmd: echo $(var simple_arg)
    output: screen
```



```bash
ros2 launch yaml_launch arg_example.launch.yaml simple_arg:="hello world"
```

### with default

```yaml title="arg_example_default.launch.yaml"
launch:
- arg:
    name: simple_arg
    default: Hello, YAML Launch!
    description: An example argument with a default value.
  
- executable:
    cmd: echo $(var simple_arg)
    output: screen
```

```bash
# list arguments
ros2 launch yaml_launch arg_example_default.launch.yaml -s

#usage
ros2 launch yaml_launch arg_example_default.launch.yaml
```

### with multiple choice

```yaml
launch:
  
- arg:
    name: arg_with_choices
    default: choice_1
    choice: [value: choice_1, value: choice_2, value: choice_3]

- executable:
    cmd: echo $(var arg_with_choices)
    output: screen
```

```bash
ros2 launch yaml_launch arg_example_default.launch.yaml arg_with_choices:=choice_2

# raise exception
ros2 launch yaml_launch arg_example_default.launch.yaml arg_with_choices:=choice_20
```

---

## Rviz

```yaml
launch:
  - arg:
      name: "rviz_config_file"
      default: "$(find-pkg-share bumperbot_bringup)/config/rviz.rviz"
  - arg:
      name: "robot_description_file"
      default: "$(find-pkg-share bumperbot_description)/urdf/bumperbot.urdf.xacro"
  - node:
      pkg: rviz2
      exec: rviz2
      name: rviz2
      output: screen
      args:
          "-d $(var rviz_config_file)"
  - node:
      pkg: "robot_state_publisher"
      exec: "robot_state_publisher"
      name: "robot_state_publisher"
      param:
        - name: "robot_description"
          value : "$(command '$(find-exec xacro) $(var robot_description_file)')"
  - node:
      pkg: "joint_state_publisher_gui"
      exec: "joint_state_publisher_gui"
      name: "joint_state_publisher_gui"
      output: screen
```

---
## Gazebo

```yaml
launch:

- executable:
    cmd: gz sim -v 4 -r empty.sdf

```