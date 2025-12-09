---
title: Gazebo custom plugin
tags:
    - gazebo
    - gz
    - jetty
    - plugin
    - custom
---

## Simple model plugin


```cpp title="simple.cpp"
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <iostream>

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
    if (_sdf->HasElement("test_value"))
    {
      auto value = _sdf->Get<std::string>("test_value");
      std::cout << "[SimpleParamReader] test_value: " << value << "\n";
    }
    else
    {
      std::cout << "[SimpleParamReader] No <test_value> found in SDF\n";
    }
  }
};

GZ_ADD_PLUGIN(SimpleParamReader,
              System,
              SimpleParamReader::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(SimpleParamReader,
                    "gz::sim::systems::SimpleParamReader")

```


```c title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.10)
project(SimpleParamReader)

find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED)

add_library(SimpleParamReader SHARED simple.cpp)

target_link_libraries(SimpleParamReader
  gz-sim::gz-sim
  gz-plugin::gz-plugin
)

install(TARGETS SimpleParamReader
  LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/bin)

```

!!! Note
    The `name` in the `plugin` section is register by
    - GZ_ADD_PLUGIN
    - GZ_ADD_PLUGIN_ALIAS 

```xml title="model plugin section"
 <plugin
    name="gz::sim::systems::SimpleParamReader"
    filename="libSimpleParamReader.so">
    <test_value>HelloWorld</test_value>
</plugin>
```


!!! Tip
    Don't forget to set `GZ_SIM_SYSTEM_PLUGIN_PATH` 