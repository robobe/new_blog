---
tags:
    - ros2
    - python
    - test
    - pytest
---

# ROS2 Python Unit Test
[base on](https://arnebaeyens.com/blog/2024/ros2-integration-testing/)
## Simple very simple demo
Show how to use pytest in ros2 python package using ament_cmake 


### package
Using `ament_cmake` to create a python package


```
├── CMakeLists.txt
├── package.xml
├── ros_py
│   ├── __init__.py
│   └── simple_node.py
└── test
    └── test_simple.py
```

### pytest test
```python title="simple_node.py"
def test_math():
    assert 2 + 2 == 4
```

### CMakeLists.txt
- Add `ament_add_pytest_test` to the CMakeLists.txt file to run the test


```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_math test/test_simple.py)
endif()
```


### using colcon

```bash
# run package test
colcon test --packages-select ros_py  --event-handlers console_direct+

# run specific test
colcon test --packages-select ros_py --pytest-args -k test_math --event-handlers console_direct+
```

---

## Reference
- [Integration and unit testing in ROS 2](https://arnebaeyens.com/blog/2024/ros2-integration-testing/)