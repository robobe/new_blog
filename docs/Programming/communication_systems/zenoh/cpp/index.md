---
title: Zenoh C++ binding
tags:
    - programming
    - communication
    - distributed-systems
    - zenoh
    - cpp
    - pub-sub
---

# Zenoh C++ binding

Zenoh has a C++ binding named `zenoh-cpp`.

Use it when a C++ application needs to publish, subscribe, query, or exchange
data with other Zenoh applications.

Typical use cases:

- robotics nodes
- telemetry
- sensor data
- distributed C++ services
- communication between C++, Python, Rust, or other Zenoh applications

## Install idea

The C++ binding depends on the Zenoh C library.

The common project shape is:

```text
C++ app
    |
    v
zenoh-cpp
    |
    v
zenoh-c
    |
    v
Zenoh network
```

Check the official install instructions for your platform before building.

## Minimal pub/sub example

This example starts one subscriber and one publisher in the same program.

The subscriber listens to:

```text
demo/cpp/**
```

The publisher sends data to:

```text
demo/cpp/counter
```

Because `demo/cpp/counter` matches `demo/cpp/**`, the subscriber receives the
messages.

```cpp
#include <chrono>
#include <iostream>
#include <thread>

#include <zenoh.hxx>

int main()
{
    auto config = zenoh::Config::create_default();
    auto session = zenoh::Session::open(std::move(config));

    auto subscriber = session.declare_subscriber(
        zenoh::KeyExpr("demo/cpp/**"),
        [](const zenoh::Sample& sample) {
            std::cout << "received "
                      << sample.get_keyexpr().as_string_view()
                      << ": "
                      << sample.get_payload().as_string()
                      << '\n';
        }
    );

    auto publisher = session.declare_publisher(
        zenoh::KeyExpr("demo/cpp/counter")
    );

    for (int counter = 0; counter < 5; ++counter) {
        std::string message = "counter=" + std::to_string(counter);

        std::cout << "publish demo/cpp/counter: " << message << '\n';
        publisher.put(message);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
```

Expected idea:

```text
publish demo/cpp/counter: counter=0
received demo/cpp/counter: counter=0
publish demo/cpp/counter: counter=1
received demo/cpp/counter: counter=1
```

## What the code does

Create default config:

```cpp
auto config = zenoh::Config::create_default();
```

Open a Zenoh session:

```cpp
auto session = zenoh::Session::open(std::move(config));
```

Declare a subscriber:

```cpp
auto subscriber = session.declare_subscriber(
    zenoh::KeyExpr("demo/cpp/**"),
    [](const zenoh::Sample& sample) {
        ...
    }
);
```

Declare a publisher:

```cpp
auto publisher = session.declare_publisher(
    zenoh::KeyExpr("demo/cpp/counter")
);
```

Send data:

```cpp
publisher.put("hello");
```

## Important classes

| C++ type | Meaning |
| --- | --- |
| `zenoh::Config` | Session configuration: mode, discovery, endpoints, transports |
| `zenoh::Session` | Connection to the Zenoh system |
| `zenoh::KeyExpr` | Key expression, such as `demo/cpp/counter` or `demo/cpp/**` |
| `zenoh::Publisher` | Object used to publish data |
| `zenoh::Subscriber` | Object that receives matching samples |
| `zenoh::Sample` | Received data sample |

## Simple CMake shape

The exact package target can depend on how `zenoh-cpp` is installed.

The project usually looks like this:

```cmake
cmake_minimum_required(VERSION 3.20)
project(zenoh_cpp_demo LANGUAGES CXX)

add_executable(zenoh_cpp_demo
    main.cpp
)

target_compile_features(zenoh_cpp_demo PRIVATE cxx_std_17)

# Use the target name provided by your zenoh-cpp installation.
# Check the official zenoh-cpp installation docs for the exact package target.
find_package(zenohcxx REQUIRED)

target_link_libraries(zenoh_cpp_demo
    PRIVATE
        zenohcxx::zenohc
)
```

Build:

```bash
cmake -S . -B build
cmake --build build
```

## Configured peer example

The default config is good for local demos.

For controlled deployment, configure endpoints explicitly.

Example idea:

```cpp
auto config = zenoh::Config::create_default();

config.insert_json5("mode", R"("peer")");
config.insert_json5("connect/endpoints", R"(["tcp/192.168.1.50:7447"])");
config.insert_json5("scouting/multicast/enabled", "false");

auto session = zenoh::Session::open(std::move(config));
```

Meaning:

- run in `peer` mode
- connect to a known endpoint
- disable multicast scouting

Use explicit endpoints when discovery is blocked or when you want deterministic
connections.

## Important topics to learn next

- `zenoh::Config` and endpoints
- peer vs client mode
- multicast discovery
- publisher lifetime
- subscriber callback lifetime
- payload encoding
- query and queryable
- router-based deployment
- CMake integration

## Reference

- [zenoh-cpp documentation](https://zenoh-cpp.readthedocs.io/)
- [zenoh-cpp publish/subscribe](https://zenoh-cpp.readthedocs.io/en/latest/pubsub.html)
- [zenoh-cpp examples](https://github.com/eclipse-zenoh/zenoh-cpp/tree/main/examples)
