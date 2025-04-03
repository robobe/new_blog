---
tags:
    - ros
    - python
    - cmake
    - ament
---

# python ament cmake

Use ament_cmake to create python package


- Create ament package
- Add folder `my_cmake_py_pkg` 
- Add `__init__.py` in `my_cmake_py_pkg` to mark as python module
- Add node python file `my_node.py`
- Set `my_node.py` as executable
- Add shabang `#!/usr/bin/env python3` in `my_node.py`
- Edit to `CMakeLists.txt`
- Edit `package.xml`


## Create package
```
ros2 pkg create my_cmake_py_pkg --build-type ament_cmake
```


## CMakeLists.txt

```cmake title="CMakeLists.txt" linenums="1" hl_lines="10 13 15-17"
cmake_minimum_required(VERSION 3.8)
project(my_cmake_py_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install python package
ament_python_install_package(${PROJECT_NAME})

set(NODES
  my_cmake_py_pkg/my_node.py
)


install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
```

### package.xml
Add `ament_cmake_python` in `buildtool_depend`

```xml 
<buildtool_depend>ament_cmake_python</buildtool_depend>
```

### python nodes
- Create folder with the package name
- Create `__init__.py` in the folder
- Every node should have `#!/usr/bin/env python3` as first line
- Make the node executable `chmod +x my_node.py`


```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        node_name="minimal"
        super().__init__(node_name)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### usage

```bash
ros2 run my_cmake_py_pkg my_node.py
```

---

### VScode tips
Install `chmod` extension to set handle executable bit on file

[chmod extension](https://marketplace.visualstudio.com/items?itemName=dlech.chmod)