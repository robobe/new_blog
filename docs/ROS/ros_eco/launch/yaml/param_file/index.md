---
title: ROS yaml launch with parameter file
tags:
  - ros
  - launch
  - yaml
---

```yaml title="config/params.yaml"
simple_param_node:
  ros__parameters:
    greeting: "Hi from YAML!"

```

```yaml
launch:
- node:
    pkg: "yaml_launch"
    exec: "simple_param_node.py"
    param:
      - from: $(find-pkg-share yaml_launch)/config/params.yaml
```


!!! Note
    We can load params from multiple file using multiple `form`