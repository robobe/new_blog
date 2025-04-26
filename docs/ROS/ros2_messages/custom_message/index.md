---
tags:
    - ros
    - interface
---

# ROS2 Custom interface

- Create `ament_cmake` package
- Create folders `srv` and `msg`
- Update `CMakeLists.txt` and `package.xml`
- VSCode tips

```bash
├── CMakeLists.txt
├── msg
│   └── Demo.msg
├── package.xml
└── srv
    └── Demo.srv
```

```title="Demo.msg"
int32 data
```

```title="Demo.srv"
---
string message
bool success
```

---

### CMakelists.txt and package.xml


```cmake title="CMakeLists.txt" linenums="1" hl_lines="10-27"
cmake_minimum_required(VERSION 3.8)
project(pkg_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MESSAGES
  "msg/Demo.msg"
)

set(SERVICES 
  "srv/Demo.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGES}
  ${SERVICES} 
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

```xml title="package.xml" linenums="1" hl_lines="12-14"
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pkg_interface</name>
  <version>0.0.1</version>
  <description>Simple interface</description>
  <maintainer email="robo2020@gmail.com">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

### VSCode tips
Fix pylance extra path

```json
"${workspaceFolder}/install/<package_name>/local/lib/python3.10/
```

```json linenums="1" hl_lines="3"
"python.analysis.extraPaths": [
        "${workspaceFolder}/",
        "${workspaceFolder}/install/pkg_interface/local/lib/python3.10/dist-packages/",
        "/opt/ros/humble/lib/python3.10/site-packages/",
        "/opt/ros/humble/local/lib/python3.10/dist-packages/"
    ]
```