---
title: Read YAML Configuration with yaml-cpp
tags:
    - cpp
    - yaml
    - yaml-cpp
    - cmake
---

`yaml-cpp` is a C++ library for parsing and generating YAML. It loads YAML data
into a tree of `YAML::Node` objects. A node may contain a mapping, sequence, or
scalar value, and `.as<T>()` converts a scalar to a C++ type.

This minimal C++17 example reads an application configuration from a file. It
does not modify or write YAML.

## Install on Ubuntu 24.04

```bash
sudo apt update
sudo apt install build-essential cmake libyaml-cpp-dev
```

Ubuntu 24.04 provides yaml-cpp `0.8.0`. Confirm the installed package with:

```bash
dpkg-query -W libyaml-cpp-dev
```

---

## Example configuration


```yaml title="config.yaml"
app:
  name: hello-yaml
  port: 8080

features:
  - reading mappings
  - reading sequences

```

The document contains:

- `app`: a mapping containing other key-value pairs;
- `name` and `port`: scalar values;
- `features`: a sequence of scalar strings.

YAML indentation defines the structure. Use spaces consistently; tab
characters are not valid indentation.

---

## Read the file in C++


```cpp title="main.cpp"
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    const std::string input_path = argc > 1 ? argv[1] : "config.yaml";
    try {
        const YAML::Node config = YAML::LoadFile(input_path);

        const std::string name = config["app"]["name"].as<std::string>();
        const int port = config["app"]["port"].as<int>();

        std::cout << "Application: " << name << '\n';
        std::cout << "Port: " << port << '\n';
        std::cout << "Features:\n";
        for (const YAML::Node& feature : config["features"]) {
            std::cout << "  - " << feature.as<std::string>() << '\n';
        }

    } catch (const YAML::Exception& error) {
        std::cerr << "YAML error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}

```

The important operations are:

```cpp
const YAML::Node config = YAML::LoadFile(input_path);
```

`LoadFile` parses the file and returns its root node.

```cpp
const std::string name = config["app"]["name"].as<std::string>();
const int port = config["app"]["port"].as<int>();
```

`operator[]` moves through mapping keys. `.as<std::string>()` and `.as<int>()`
convert YAML scalars into the requested C++ types.

```cpp
for (const YAML::Node& feature : config["features"]) {
    std::cout << feature.as<std::string>() << '\n';
}
```

A YAML sequence can be traversed with a range-based `for` loop.

!!! warning "Parsing and conversion can fail"
    `YAML::LoadFile` throws when the file cannot be opened or parsed. `.as<T>()`
    throws when a required value is missing or cannot be converted to `T`. The
    example catches `YAML::Exception`, prints the error, and exits with a
    non-zero status.

---

## CMake configuration

[Download `CMakeLists.txt`](code/CMakeLists.txt).

```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.16)

project(yaml_read_demo LANGUAGES CXX)

find_package(yaml-cpp REQUIRED)

add_executable(read_yaml main.cpp)
target_link_libraries(read_yaml PRIVATE yaml-cpp::yaml-cpp)
target_compile_features(read_yaml PRIVATE cxx_std_17)
target_compile_options(read_yaml PRIVATE -Wall -Wextra -pedantic)

# Copy the example configuration into the build directory.
configure_file(config.yaml config.yaml COPYONLY)

```

`find_package` locates the Ubuntu development package. The imported target
supplies the include directories and library required by `read_yaml`:

```cmake
find_package(yaml-cpp REQUIRED)
target_link_libraries(read_yaml PRIVATE yaml-cpp::yaml-cpp)
```

`configure_file` copies `config.yaml` into the build directory, making the
default relative path work when the executable is launched there.

---

## Build and run

Download the three files into one directory, then run:

```bash
cmake -S . -B build
cmake --build build
cd build
./read_yaml
```

Expected output:

```text
Application: hello-yaml
Port: 8080
Features:
  - reading mappings
  - reading sequences
```



---

## Demo: Map to struct

The first example accesses each YAML node directly. That is useful for small
files, but configuration code becomes difficult to maintain when the same keys
are read in many places. A typed configuration keeps the rest of the program
independent from the YAML tree:

```cpp
struct AppConfig {
    std::string name;
    int port;
};

struct Config {
    AppConfig app;
    std::vector<std::string> features;
};
```

These structures mirror the document:

```text
config.yaml                  C++
-----------                  ---
app                 ->       Config::app
  name              ->       AppConfig::name
  port              ->       AppConfig::port
features             ->       Config::features
```

### Define a yaml-cpp conversion

yaml-cpp already knows how to convert scalars and standard containers such as
`std::vector<std::string>`. It does not know the meaning of application types
such as `AppConfig`, so the example specializes `YAML::convert<T>`:

```cpp
namespace YAML {

template<>
struct convert<AppConfig> {
    static bool decode(const Node& node, AppConfig& value)
    {
        if (!node.IsMap() || !node["name"] || !node["port"]) {
            return false;
        }

        value.name = node["name"].as<std::string>();
        value.port = node["port"].as<int>();
        return true;
    }
};

} // namespace YAML
```

`decode` performs three jobs:

1. Confirm that the node is a mapping.
2. Confirm that required keys exist.
3. Convert each value into the corresponding structure member.

Returning `false` tells yaml-cpp that the node cannot be converted to the
requested type. Scalar conversions such as `.as<int>()` can also throw when a
value has the wrong type.

`Config` has its own conversion and can reuse the nested conversion:

```cpp
value.app = node["app"].as<AppConfig>();
value.features = node["features"].as<std::vector<std::string>>();
```

The complete YAML document can then be converted with one expression:

```cpp
const Config config = YAML::LoadFile(input_path).as<Config>();
```

After this line, normal application code uses `config.app.name` and
`config.features`; it no longer needs to know YAML key paths.

!!! tip "Keep parsing at the application boundary"
    Convert YAML into typed configuration objects near program startup. Pass
    those objects to the rest of the application instead of passing
    `YAML::Node` everywhere. This keeps yaml-cpp details out of business logic.

!!! warning "Validate meaning separately"
    Successful type conversion does not prove that a value is sensible. Add
    application checks for rules such as `1 <= port <= 65535`, non-empty names,
    and permitted feature values.

### Complete example

[Download `map_yaml_to_struct.cpp`](code/map_yaml_to_struct.cpp).

```cpp title="map_yaml_to_struct.cpp"
--8<-- "docs/Programming/cpp/libraries/yaml_cpp/code/map_yaml_to_struct.cpp"
```

The page's `CMakeLists.txt` builds this as a second executable:

```cmake
add_executable(map_yaml_to_struct map_yaml_to_struct.cpp)
target_link_libraries(map_yaml_to_struct PRIVATE yaml-cpp::yaml-cpp)
target_compile_features(map_yaml_to_struct PRIVATE cxx_std_17)
```

Build and run it from the downloaded example directory:

```bash
cmake -S . -B build
cmake --build build
./build/map_yaml_to_struct config.yaml
```

It produces the same output as `read_yaml`, but the parsed data is stored in
typed C++ structures.


---

## Demo: Save mapped struct back to YAML file

The previous example implemented only `decode`, which supports this direction:

```text
YAML node --decode--> C++ structure
```

Implementing `encode` adds the reverse direction:

```text
YAML file --decode--> C++ structure --update--> C++ structure
                                             --encode--> YAML node --save--> YAML file
```

### Add `encode` to each mapped type

`encode` creates a new `YAML::Node` and assigns each structure member to the
corresponding YAML key:

```cpp
static YAML::Node encode(const AppConfig& value)
{
    YAML::Node node;
    node["name"] = value.name;
    node["port"] = value.port;
    return node;
}
```

The outer `Config` conversion can assign the nested structure directly:

```cpp
static YAML::Node encode(const Config& value)
{
    YAML::Node node;
    node["app"] = value.app;
    node["features"] = value.features;
    return node;
}
```

When `node["app"] = value.app` is evaluated, yaml-cpp finds the
`YAML::convert<AppConfig>::encode` specialization. It already knows how to
encode `std::vector<std::string>`.

### Read, update, and save

First load the YAML file into a mutable structure:

```cpp
Config config = YAML::LoadFile(input_path).as<Config>();
```

The application then changes ordinary C++ members without working directly
with YAML nodes:

```cpp
config.app.name = "updated-yaml-demo";
config.app.port = 9090;
config.features.push_back("struct-mapping");
```

Finally, encode the complete structure and send the resulting node to an output
stream:

```cpp
const YAML::Node output_node = YAML::convert<Config>::encode(config);
std::ofstream output(output_path);
if (!output) {
    std::cerr << "Cannot open output file: " << output_path << '\n';
    return 1;
}
output << output_node;
```

!!! warning "Encoding reconstructs the document"
    This method creates a new YAML document from the mapped fields. Comments,
    original formatting, key order, and unknown keys are not guaranteed to be
    preserved. Use it for application-owned configuration rather than as a
    general-purpose editor for human-authored YAML.

!!! tip "Write to a separate file first"
    The example defaults to `struct_updated.yaml`, leaving `config.yaml`
    unchanged. For important configuration, write a temporary file, verify the
    stream succeeded, and atomically replace the destination only after the
    complete document is safely written.

### Complete example

[Download `update_struct_to_yaml.cpp`](code/update_struct_to_yaml.cpp).

```cpp title="update_struct_to_yaml.cpp"
--8<-- "docs/Programming/cpp/libraries/yaml_cpp/code/update_struct_to_yaml.cpp"
```

The page's `CMakeLists.txt` builds a third executable:

```cmake
add_executable(update_struct_to_yaml update_struct_to_yaml.cpp)
target_link_libraries(update_struct_to_yaml PRIVATE yaml-cpp::yaml-cpp)
target_compile_features(update_struct_to_yaml PRIVATE cxx_std_17)
```

Build and run it from the downloaded example directory:

```bash
cmake -S . -B build
cmake --build build
./build/update_struct_to_yaml config.yaml struct_updated.yaml
```

The generated file contains:

```yaml title="struct_updated.yaml"
app:
  name: updated-yaml-demo
  port: 9090
features:
  - reading mappings
  - reading sequences
  - struct-mapping
```


---

## Reference

- [yaml-cpp project](https://github.com/jbeder/yaml-cpp){:target="_blank" rel="noopener noreferrer"}
- [yaml-cpp tutorial](https://github.com/jbeder/yaml-cpp/wiki/Tutorial){:target="_blank" rel="noopener noreferrer"}

<!-- post-content-skill: 1.0.0 -->
