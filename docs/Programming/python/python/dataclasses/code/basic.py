from dataclasses import dataclass
from typing import Annotated
from typing import get_type_hints, get_origin, get_args
import struct

@dataclass
class Packet:
    a: Annotated[int, "h", "xx"]   # int16
    b: Annotated[int, "I"]   # uint32
    c: Annotated[float, "f"] # float32
    name: Annotated[str, "16s"]

def struct_format(cls, endian="<"):
    hints = get_type_hints(cls, include_extras=True)
    fmt = endian

    for name in cls.__dataclass_fields__:
        t = hints[name]
        if get_origin(t) is Annotated:
            print(get_args(t))
            _, fmt_char, *_ = get_args(t)
            fmt += fmt_char
        else:
            raise TypeError(f"Field {name} missing struct annotation")

    return fmt

fmt = struct_format(Packet)
print(fmt)  # "<hIf16s"