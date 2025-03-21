---
tags:
    - ros
    - plugins
    - pluginlib
    - cpp
---

# ROS Pluginlib

Pluginlib is a C++ library for loading and unloading plugins from within a ROS package. Plugins are dynamically loadable classes that are loaded from a runtime library (shared object)


## Demo
Demo how to use pluginlib , the demo include 3 packages:

- **polygon_base**: Interface declaration
- **polygons**: Plugin implementation
- **polygons_app**: Plugin usage


### Package setup

```bash
ros2 pkg create polygon_base --build-type ament_cmake
ros2 pkg create polygons --build-type ament_cmake --dependencies pluginlib polygon_base
ros2 pkg create polygons_app --build-type ament_cmake --dependencies pluginlib polygon_base
```

### polygon_base
polygon_base expose the abstract class that the plugin need to implement  
The plugin constructor need to haven't any parameters and for plugin initialization we use the `init` method


```cpp title="include/polygon_base/polygon.hpp"
#pragma once

namespace PolygonBase
{
    class Polygon
    {
    public:
        //! Necessary since constructor can't have parameters!
        virtual void init(double side_length) = 0; // 1
        virtual double area() = 0;
        virtual ~Polygon() {}

    protected:
        //! This costructor signature is required by pluginlib
        Polygon() {}
    };

} // namespace PolygonBase
```

Build the package and expose the `include` folder downstream to other packages

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(polygon_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY include/  #(1)
  DESTINATION include)
  
ament_export_include_directories(include) #(2)

ament_package()

```

1. Copy the include folder to installation folder
2. **ament_export_include_directories**: exports your package’s include directories (e.g., include/) to downstream packages, making headers like provider_pkg/provider.hpp discoverable via ${provider_pkg_INCLUDE_DIRS}.

---

### polygons
     
Implement plugins and build them as shared library and expose the to **pluginlib** infa using xml declaration

!!! note "package.xml"
    Don't forget to insert package dependencies
    ```xml
    <depend>polygon_base</depend>
    <depend>pluginlib</depend>
    ```
     
```
├── CMakeLists.txt
├── include
│   └── polygons
│       └── square.hpp
├── package.xml
├── plugins_triangle.xml
├── plugins.xml
└── src
    ├── square.cpp
    └── triangle.cpp
```

```cpp title="src/triangle.cpp"
 #include <cmath>
 #include <polygon_base/polygon.hpp>
 
 namespace Polygons
 {
 
    class Triangle : public PolygonBase::Polygon
    {
        void init(double side_length)
        {
        side_length_ = side_length;
        }
 
 
        double area()
        {
        return 0.5 * side_length_ * compute_height();
        }
 
        
        double compute_height()
        {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2.0) * (side_length_ / 2.0)));
        }

        private: // (2) best practice to declare private as protected, but it's ok for basic types
            double side_length_;
    };
 
 }  // namespace Polygons

#include <pluginlib/class_list_macros.hpp>  //(1)
PLUGINLIB_EXPORT_CLASS(Polygons::Triangle, PolygonBase::Polygon)
```

1. Register the plugin in **Pluginlib** infra
2. **Best practice** to use internal variable as protected (Private members might cause issues in dynamic loading scenarios.)
     
```xml title="plugins_triangle.xml"
<library path="triangle"> 
  <class type="Polygons::Triangle" base_class_type="PolygonBase::Polygon">
    <description>This is a Triangle plugin.</description>
  </class>
</library>
```

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(polygons)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(polygon_base REQUIRED)
find_package(pluginlib REQUIRED)


add_library(triangle SHARED src/triangle.cpp)
target_compile_features(triangle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  triangle
  pluginlib
  polygon_base)


install(
  TARGETS triangle
  EXPORT export_${PROJECT_NAME} #(2)
  LIBRARY DESTINATION lib)


pluginlib_export_plugin_description_file(polygon_base plugins_triangle.xml) #(1)


ament_package()

```

1. The macro PLUGINLIB_EXPORT_PLUGIN_DESCRIPTION_FILE is used in ROS 2 pluginlib to specify the plugin **XML file that describes available plugins**. This macro helps the pluginlib system locate and register plugins when loading them dynamically.
2. The install() command in CMake is used to define installation rules for compiled targets (executables, libraries, etc.). The EXPORT keyword allows us to export a CMake target so that other projects can easily find and use it.

#### Square Plugin
Add square plugin, for the purpose of the demo build it as separate library (so)



<details>
  <summary>Header file</summary>

  ```cpp
  #pragma once

  #include <polygon_base/polygon.hpp>

  //! Namespaces must reflect what is written in plugins.xml!
  namespace Polygons
  {

      /**
       * Regular polygon with four sides.
       */
      class Square : public PolygonBase::Polygon
      {
      public:
          void init(double side_length) override;
          double area() override;

      //! There must not be any private members (best practice)
      protected:
          double side_length_;
      };

  } // namespace Polygons

  //! Necessary to register plugin classes with pluginlib
  #include <pluginlib/class_list_macros.hpp>
  PLUGINLIB_EXPORT_CLASS(Polygons::Square, PolygonBase::Polygon)
  ```
</details>


<details>
  <summary>plugin cpp file</summary>

```cpp title="square.cpp"
#include <cmath>

#include "polygons/square.hpp"

namespace Polygons
{

    void Square::init(double side_length)
    {
        side_length_ = side_length;
    }

    double Square::area()
    {
        return pow(side_length_, 2.0);
    }

}
```
</details>

<details>
  <summary>Plugin xml</summary>

```xml title="plugins.xml"
<library path="square">
  <class type="Polygons::Square" base_class_type="PolygonBase::Polygon">
    <description>This is a square plugin.</description>
  </class>
</library>

```
</details>

<details>
  <summary>Full CMakeLists.txt</summary>

```cmake
cmake_minimum_required(VERSION 3.8)
project(polygons)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(polygon_base REQUIRED)
find_package(pluginlib REQUIRED)

# square library
add_library(square SHARED src/square.cpp)
target_compile_features(square PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(square PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  square
  pluginlib
  polygon_base)

# square library
add_library(triangle SHARED src/triangle.cpp)
target_compile_features(triangle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  triangle
  pluginlib
  polygon_base)

install(
    TARGETS square
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib)

install(
  TARGETS triangle
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)

#! link plugins with base class for pluginlib
pluginlib_export_plugin_description_file(polygon_base plugins.xml)
pluginlib_export_plugin_description_file(polygon_base plugins_triangle.xml)


ament_package()

```
</details>



---

## Polygon_app

ROS package that load the plugin dynamically and usage it

```cpp title="src/main.cpp"
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <polygon_base/polygon.hpp>

#define UNUSED(arg) (void)(arg)

int main(int argc, char ** argv)
{
  UNUSED(argc);
  UNUSED(argv);
  
  try
  {
    pluginlib::ClassLoader<PolygonBase::Polygon> loader("polygon_base", "PolygonBase::Polygon");

    std::shared_ptr<PolygonBase::Polygon> square = loader.createSharedInstance("Polygons::Square");
    std::shared_ptr<PolygonBase::Polygon> triangle = loader.createSharedInstance("Polygons::Triangle");

    triangle->init(10.0);
    square->init(10.0);

    std::cout << "Square area: " << square->area() << std::endl;
    std::cout << "Triangle area: " << triangle->area() << std::endl;
  }
  catch(const pluginlib::PluginlibException & e)
  {
    std::cerr << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }


  exit(EXIT_SUCCESS);
}
```

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.8)
project(polygons_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)

add_executable(polygons_tester src/main.cpp)
target_compile_features(polygons_tester PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  polygons_tester
  pluginlib
  polygon_base)

install(TARGETS polygons_tester
  DESTINATION lib/${PROJECT_NAME})

ament_package()

```

The package not know about the `plugins` package it loaded dynamic by the pluginlib.  
It only reference to the package with the abstract class


```xml title="package.xml"
  <depend>pluginlib</depend>
  <depend>polygon_base</depend>
```

---

### Usage

```bash
ros2 run polygons_app polygons_tester 
#
Square area: 100
Triangle area: 43.3013
```