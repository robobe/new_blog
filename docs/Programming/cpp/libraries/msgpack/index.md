---
title: Using MsgPack in CPP
tags:
    - cpp
    - msgpack
---


{{ page_folder_links() }}

## Install

```
sudo apt install libmsgpack-dev
```

---
[magpack-c](https://github.com/msgpack/msgpack-c/wiki/v2_0_cpp_tutorial)

## Demo:

!!! note "Header only"
     

```cpp
--8<-- "docs/Programming/cpp/libraries/msgpack/code/demo.cpp"
```

```
g++ -I/usr/include demo.cpp -o demo
```

#### cmake

```cmkae
--8<-- "docs/Programming/cpp/libraries/msgpack/code/CMakeLists.txt"
```

### pack struct

```cpp
--8<-- "docs/Programming/cpp/libraries/msgpack/code/pack_struct.cpp"
```

!!! tip "MSGPACK_DEFINE vs MSGPACK_DEFINE_MAP"
    MSGPACK_DEFINE and MSGPACK_DEFINE_MAP are macros that generate serialization / deserialization code for your struct or class — but they encode the data in different MessagePack formats.

    - MSGPACK_DEFINE → array-based encoding
    - MSGPACK_DEFINE_MAP → map-based encoding

    When sending message between cross language like cpp-python it best to use **MSGPACK_DEFINE_MAP**
    