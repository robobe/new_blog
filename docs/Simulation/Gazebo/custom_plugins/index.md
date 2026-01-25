---
tags:
    - harmonic
    - custom 
    - plugin
---

# Gazebo harmonic custom plugin

![](../../../assets/images/under_construction.png)


```
├── .devcontainer
│   ├── Dockerfile
│   └── devcontainer.json
├── docker-compose.yaml
├── .gitignore
└── worlds
    └── mini.world
└── models
    └── simple_box
        └── model.config
        └── model.sdf
└── bin
└── src
    └── simple
        └── simple.cc
└── CMakeLists.txt
```

```cpp title="simple.cpp"
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <iostream>
#include <gz/common/Console.hh>

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class SimpleParamReader
    : public System,
      public ISystemConfigure
{
public:
  void Configure(const Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &,
                 EventManager &) override
  {
    std::cout << "----------------------------- 0.2 ---------------------------------------" << std::endl;
    if (_sdf->HasElement("test_value"))
    {
      auto value = _sdf->Get<std::string>("test_value");
      gzmsg << "[SimpleParamReader] test_value: " << value  << std::endl;
    }
    else
    {
      gzerr << "[SimpleParamReader] No <test_value> found in SDF"  << std::endl;
    }

    gzmsg << "Hello (info)" << std::endl;
    gzwarn << "Something looks odd" << std::endl;
    gzerr << "Something failed!" << std::endl;
    gzdbg << "Debug detail: x=" << 42 << "\n" << std::endl;
    gzlog << "Trace detail" << std::endl;

    std::cout << "--------------------------------------------------------------------" << std::endl;
  }
};

GZ_ADD_PLUGIN(SimpleParamReader,
              System,
              SimpleParamReader::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(SimpleParamReader,
                    "gz::sim::systems::SimpleParamReader")
```

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.10)
project(SimpleParamReader)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(SimpleParamReader SHARED src/simple/simple.cc)

target_link_libraries(SimpleParamReader
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

install(TARGETS SimpleParamReader
  LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/bin)
```


```xml
<?xml version="1.0"?>
<sdf version="1.7">
    <model name="simple_box">
        <static>false</static>

        <link name="link">
        ...
        </link>

        <plugin
            name="gz::sim::systems::SimpleParamReader"
            filename="libSimpleParamReader.so">
            <test_value>HelloWorld</test_value>
        </plugin>

    </model>
</sdf>
```

---

### VSCode 

```json title=".vscode/settings.json"
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/include/gz/plugin2",
                "/usr/include/gz/sim8",
                "/usr/include/gz/utils2",
                "/usr/include/gz/msgs10",
                "/usr/include/gz/common5",
                "/usr/include/gz/math7",
                "/usr/include/gz/transport13",
                "/usr/include/gz/sdformat14"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}

```

### Build and install

```bash
cmake -S . -B build
cmake --build build
cmake --install build
```

!!! tip "GZ_SIM_SYSTEM_PLUGIN_PATH"

    ```bash
    export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/bin:$GZ_SIM_SYSTEM_PLUGIN_PATH
    ```