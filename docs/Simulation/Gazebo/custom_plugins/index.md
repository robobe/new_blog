---
tags:
    - harmonic
    - custom 
    - plugin
---

# Gazebo harmonic custom plugin

[gazebo examples](https://github.com/gazebosim/gz-sim/tree/gz-sim10/examples)

![](../../../assets/images/under_construction.png)


```
Plugin library loaded
      |
      v
+------------------------------+
|  Constructor (C++ object)    |
+------------------------------+
      |
      v
+---------------------------------------------------+
| Configure()  [ISystemConfigure]                   |
| - runs ONCE when plugin is attached/loaded        |
| - read/write EntityComponentManager (ECM)         |
| - parse SDF params, create components, cache IDs  |
+---------------------------------------------------+
      |
      v
      Simulation loop (every tick / iteration)
      |
      v
+---------------------------------------------------+
| PreUpdate()  [ISystemPreUpdate]                   |
| - runs EVERY tick, BEFORE physics step            |
| - read/write ECM                                 |
| - apply commands (forces, joint targets, etc.)    |
+---------------------------------------------------+
      |
      v
+---------------------------------------------------+
| Update()     [ISystemUpdate] (optional)           |
| - runs EVERY tick, during the "update" phase      |
| - read/write ECM                                 |
| - typically physics/stepping related systems      |
+---------------------------------------------------+
      |
      v
+---------------------------------------------------+
| PostUpdate() [ISystemPostUpdate]                  |
| - runs EVERY tick, AFTER physics step             |
| - READ-ONLY ECM (important!)                      |
| - publish state, compute observations/feedback    |
+---------------------------------------------------+
      |
      v
(loop back to PreUpdate for next tick)
      |
      v
+---------------------------------------------------+
| Reset()      [ISystemReset] (optional)            |
| - runs when the sim/model/plugin is reset         |
| - clear integrators, counters, internal state     |
+---------------------------------------------------+
      |
      v
+------------------------------+
| Destructor (plugin unloaded) |
+------------------------------+

```

### Rule of thumb
- Control / write to world: **PreUpdate()**
- Observe / publish state: **PostUpdate()**
- One-time setup: **Configure()**
- Clear internal state: **Reset()**

---

## ECM (Entity Component Manager)

Instead of objects with C++ classes (like Classic Gazebo), Gazebo Sim uses an **Entity–Component–System** (ECS) architecture.

- **Entity** = integer ID
- **Component** = data attached to entity
- **System (plugin)** = logic that reads/writes components


### Entity
An Entity is just an ID (no methods, no class).

Examples of entities:

- World
- Model
- Link
- Joint
- Sensor

### Components
Components are typed data blobs attached to entities.

| Component      | Meaning                |
| -------------- | ---------------------- |
| Pose           | Position + orientation |
| LinearVelocity | Velocity               |
| JointPosition  | Joint angle            |
| Name           | Entity name            |
| ParentEntity   | Parent in tree         |
| Link           | Marks entity as a link |


### System

A System (plugin) reads/writes components via the ECM.

### Simple demo:
- Read Model pose
- Get entity in configure method
- Read entity pose component every simulation step

```cpp
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/common/Console.hh>
#include <gz/sim/Entity.hh>

using namespace gz;
using namespace sim;

class SimpleECM :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    modelEntity = entity;
    gzmsg << "Plugin attached to " << entity << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &ecm) override
  {
    
    // Read pose component from ecm for entity modelEntity
    auto poseComp = ecm.Component<gz::sim::components::Pose>(modelEntity);
    if (!poseComp) return;

    auto pose = poseComp->Data();
    gzmsg << "-----> Z = " << pose.Pos().Z() << "\n";
  }

private:
  gz::sim::Entity modelEntity{kNullEntity};
};

GZ_ADD_PLUGIN(SimpleECM, gz::sim::System,
              SimpleECM::ISystemConfigure,
              SimpleECM::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SimpleECM,
                    "gz::sim::systems::SimpleECM")

```

---

<div class="grid-container">
    <div class="grid-item">
        <a href="pub_sub">
        <p>Pub/Sub</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="imu_sensor">
        <p>imu</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="debugging">
        <p>Debugging</p>
        </a>
    </div>
</div>


---

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