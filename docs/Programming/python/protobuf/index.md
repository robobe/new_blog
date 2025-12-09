---
title: Python protobuf 
tags:
    - python
    - protobuf
    - protoc
---

## Install

```bash
sudo apt install protobuf-compiler

pip install protobuf
```

---

## Simple demo

```proto title="proto/person.proto"
syntax = "proto3";

message Person {
  string name = 1;
  int32 id = 2;
  string email = 3;
}
```

- Import and create person class
- Serialize
- Deserialization `ParseFromString`

```python
from msgs import person_pb2

# Create a Person object
person = person_pb2.Person()
person.name = "Alice"
person.id = 123
person.email = "alice@example.com"

# Serialize to bytes
data = person.SerializeToString()

# Deserialize
new_person = person_pb2.Person()
new_person.ParseFromString(data)

print(f"Name: {new_person.name}, ID: {new_person.id}, Email: {new_person.email}")

```


---

## Python intellisense

```bash
pip install mypy-protobuf
```

- Create `pyi` file for 
```bash
protoc \
--python_out=msgs \
--mypy_out=msgs \
--proto_path=proto person.proto
```