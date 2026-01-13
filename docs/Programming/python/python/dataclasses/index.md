---
title: Python dataclass 
tags:
    - python
    - dataclass
---
A dataclass is a Python class that is mainly used to store data, and Python automatically generates:

- `__init__`
- `__repr__`
- `__eq__`


```python title="basic"
from dataclasses import dataclass

@dataclass
class Point:
    x: float
    y: float
```

```python title="usage"
p = Point(1.0, 2.0)
print(p)
```

---

### Export as dict or tuple

```
from dataclasses import dataclass,asdict, astuple

@dataclass
class Point:
    x: float
    y: float

p = Point(1.0, 2.0)
print(asdict(p))
print(astuple(p))
```

---

## post_init
`__post_init__` is a hook that runs immediately after the dataclass-generated`__init__` finishes.

### usage demo

- r : is a calculate field

```python
from dataclasses import dataclass, field

@dataclass
class Vector:
    x: float
    y: float
    r: float = field(init=False)

    def __post_init__(self):
        self.r = (self.x**2 + self.y**2) ** 0.5
```


---

## field 

You use field() when you want extra control over a property:

- default values
- default factory (for lists, dicts, etc.)
- exclude from __init__
- exclude from __repr__
- metadata
- comparison behavior

```
attribute: type = field(...)
```

### default_factory
init the mutable attribute correctly 
Use default_factory for:
- list
- dict
- set
- custom objects

```python
@dataclass
class GoodExample:
    items: list = field(default_factory=list)
```

### metadata
attach extra info to fields:

```python
@dataclass
class Config:
    port: int = field(metadata={"unit": "tcp"})

```

```python
Config.__dataclass_fields__["port"].metadata
# return metadata as dictionary 
```



---

## Annotation

```python
from dataclasses import dataclass
from typing import Annotated

@dataclass
class Sensor:
    angle: Annotated[float, "deg", "imu_angle"]
    data: Annotated[float, "deg", "other data"] = field(default=100)

```

### Extract parts of Annotated

```python
from typing import get_origin, get_args
from typing import get_type_hints


hints = get_type_hints(Sensor, include_extras=True)
annot = hints["angle"]

print(get_origin(annot))  # Annotated
print(get_args(annot))    # (float, 'deg', 'imu')

```

### field() vs Annotated — when to use which?

| Use case                   | Best choice             |
| -------------------------- | ----------------------- |
| Default values / factories | `field()`               |
| Hide from repr/init        | `field()`               |
| Validation constraints     | `Annotated`             |
| Serialization hints        | `Annotated` or metadata |
| Framework integration      | `Annotated`             |




---

### Demo
Using `dataclass` and annotation to build `struct` fmt dynamic from dataclass fields

```python
from dataclasses import dataclass
from typing import Annotated

@dataclass
class Packet:
    a: Annotated[int, "h"]   # int16
    b: Annotated[int, "I"]   # uint32
    c: Annotated[float, "f"] # float32
    name: Annotated[str, "16s"]


```

```python
from typing import get_type_hints, get_origin, get_args
import struct

def struct_format(cls, endian="<"):
    hints = get_type_hints(cls, include_extras=True)
    fmt = endian

    for name in cls.__dataclass_fields__:
        t = hints[name]

        if get_origin(t) is Annotated:
            _, fmt_char, *_ = get_args(t)
            fmt += fmt_char
        else:
            raise TypeError(f"Field {name} missing struct annotation")

    return fmt

```

```python
fmt = struct_format(Packet)
print(fmt)  # "<hIf16s"

```

```python title="code"
from dataclasses import dataclass, astuple
from typing import Annotated
from typing import get_type_hints, get_origin, get_args
import struct
from functools import cache

@dataclass
class Packet:
    a: Annotated[int, "h"]   # int16
    b: Annotated[int, "I"]   # uint32
    c: Annotated[float, "f"] # float32
    # name: Annotated[str, "16s"]

    
    @classmethod
    @cache
    def struct_format(cls, endian="<") -> str:
        """
        Docstring for struct_format
        
        :param cls: Class to generate format for
        :param endian: Endianness of the struct format
        """
        hints = get_type_hints(cls, include_extras=True)
        fmt = endian

        for name in cls.__dataclass_fields__:
            t = hints[name]

            if get_origin(t) is Annotated:
                _, fmt_char, *_ = get_args(t)
                fmt += fmt_char
            else:
                raise TypeError(f"Field {name} missing struct annotation")

        return fmt
    
    def pack(self, endian="<") -> bytes:
        """
        Return data packed as bytes
        
        :param endian: Endianness of the struct format
        :return: Descriptionreturn buffer as bytes
        :rtype: bytes
        """
        fmt = self.struct_format(endian)
        return struct.pack(fmt, *astuple(self))
    
    @classmethod
    def from_bytes(cls, data: bytes, endian: str="<") -> "Packet":
        """
        convert bytes to class instance

        :param cls: Class to generate format for
        :param data: buffer to unpack
        :type data: bytes
        :param endian: Endianness of the struct format
        """
        fmt = cls.struct_format(endian)
        unpacked_data = struct.unpack(fmt, data)
        return cls(*unpacked_data)


# usage
packet = Packet(a=1, b=42, c=3.14)
#pack
buffer = packet.pack()
# print(Packet.struct_format.cache_info())

#unpack
unpacked_data = Packet.from_bytes(buffer)
print(unpacked_data)
```