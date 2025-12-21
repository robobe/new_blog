---
tags:
    - xacro
    - arg
    - condition
---

# XACRO arg and condition

## arg
Simple `xacro` file that get argument from outside.  
if no argument set it use the default.  
Using `xacro` command to substitute xacro sentence

```xml title="simple.xacro"
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <xacro:arg name="banner" default="default banner"/>
    <tag>$(arg banner)</tag>
</robot>
```

```bash title="usage"
# use default value
xacro simple.xacro

# set arg from cli
xacro simple.xacro banner:=hello

```

#### launch file

##### yaml

```yaml
launch:
  - arg:
      name: "robot_description_file"
      default: "$(find-pkg-share my_robot_description)/urdf/test.xacro"
  - arg:
      name: banner
      default: hello world

  #xacro
  - executable:
      cmd: "xacro $(var robot_description_file) banner:=$(var banner)"
      output: screen
```



---

## Condition

```xml title="simple xacro with condition and argument"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
    <xacro:arg name="use_ros2_control" default="false" />
    <xacro:if value="$(arg use_ros2_control)">
        control
    </xacro:if>
    <xacro:unless value="$(arg use_ros2_control)">
        simple
    </xacro:unless>
</robot>
```

#### usage
#### launch

```yaml
launch:
  - arg:
      name: "robot_description_file"
      default: "$(find-pkg-share my_robot_description)/urdf/test.xacro"
  - arg:
      name: use_control
      default: "false"

  #xacro
  - executable:
      cmd: "xacro $(var robot_description_file) use_ros2_control:=$(var use_control)"
      output: screen
```

#### usage

```bash
#default
ros2 launch my_robot_bringup test.launch.yaml
# False
ros2 launch my_robot_bringup test.launch.yaml use_control:=false
# True
ros2 launch my_robot_bringup test.launch.yaml use_control:=false
```

---

## Demo: more conditions
Use expression to evaluate condition

```xml title="test.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
    <xacro:arg name="xxx" default="2" />
    <xacro:property name="xxx_value" value="$(arg xxx)" /> 

    arg: $(arg xxx) , property: ${xxx_value} 
    
    <xacro:if value="${'$(arg xxx)' == '1'}">
        arg condition 1
    </xacro:if>

    <xacro:if value="${xxx_value == 1}">
        property condition 1
    </xacro:if>

</robot>
```

### usage

```bash
xacro test.xacro xxx:=1
xacro test.xacro xxx:=2
```