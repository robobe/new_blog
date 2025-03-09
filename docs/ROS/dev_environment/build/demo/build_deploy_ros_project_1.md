---
tags:
    - ros
    - deploy
    - tutorial
---

# ROS2 project from development to deployment, Part 1: ROS project

The demo project has three package  
- **pkg_interface**: message and service definition  
- **pkg_server**: Expose a service that use the message from pkg_interface  
- **pkg_client**: Call the service from pkg_server  

!!! note source code
    The source code is available at [
    ![GitHub](https://badgen.net/badge/icon/GitHub?icon=github&label)](https://github.com/robobe/deploy_demo_ws)
     
---

## pkg_interface

```
.
├── CMakeLists.txt
├── msg
│   └── Demo.msg
├── package.xml
└── srv
    └── Demo.srv
```

<details>
    <summary>CMakeLists.txt</summary>

```cmake title="CMakeLists.txt" hl_lines="10-29"
{{PLUGIN uri="https://raw.githubusercontent.com/robobe/deploy_demo_ws/refs/heads/master/src/pkg_interface/CMakeLists.txt"}}
```
</details>


<details>
    <summary>package.xml</summary>

```xml title="package.xml" hl_lines="12-14"
{{PLUGIN uri="https://raw.githubusercontent.com/robobe/deploy_demo_ws/refs/heads/master/src/pkg_interface/package.xml"}}
```
</details>

<details>
    <summary>Demo.srv</summary>

```xml title="Demo.srv"
{{PLUGIN uri="https://raw.githubusercontent.com/robobe/deploy_demo_ws/refs/heads/master/src/pkg_interface/srv/Demo.srv"}}
```
</details>

---

## pkg_server

```bash
├── CMakeLists.txt
├── package.xml
└── pkg_server
    ├── __init__.py
    └── my_node.py
```

<details>
    <summary>node</summary>

```python title="my_node.py"
{{PLUGIN uri="https://raw.githubusercontent.com/robobe/deploy_demo_ws/refs/heads/master/src/pkg_server/pkg_server/my_node.py"}}
```
</details>

---

## pkg_client

```bash
├── CMakeLists.txt
├── launch
│   └── client_server.launch.py
├── package.xml
└── pkg_client
    ├── __init__.py
    └── my_node.py
```

<details>
    <summary>client node</summary>

```python title="my_node.py"
{{PLUGIN uri="https://raw.githubusercontent.com/robobe/deploy_demo_ws/refs/heads/master/src/pkg_client/pkg_client/my_node.py"}}
```
</details>


---



