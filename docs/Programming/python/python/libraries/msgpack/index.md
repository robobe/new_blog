---
title: msgpack
tags:
    - python
    - msgpack
    - serialization
---

MessagePack (msgpack) is a binary serialization format — like JSON, but:

- More compact
- Faster to serialize / deserialize
- Language-independent
- Designed for machines (not humans)

it convert Python objects → bytes.


```bash
pip install msgpack
```

---

## Usage

### Serialize (Pack)

```python
import msgpack

data = {
    "x": 10,
    "y": 20,
    "name": "robot"
}

packed = msgpack.packb(data)

print(packed)
print(type(packed))  # bytes
```

### Deserialize (Unpack)

```python
unpacked = msgpack.unpackb(packed)

print(unpacked)
print(type(unpacked))
```

---

## Supported types

MessagePack supports:

- int
- float
- str
- bytes
- list
- dict
- bool
- None

NOT Support Automatically

- Custom classes
- Numpy arrays
- Complex objects

---

## Demo: Custom encode / decode
- encode dataclass fields as list
- add encode and decode method to the class
- add tagtype and version as ClassVar fields

<details>
<summary>Demo code</summary>
```
--8<-- "code/custom.py"
```
</details>