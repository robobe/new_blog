from dataclasses import dataclass, field
from typing import Annotated
from typing import get_type_hints

@dataclass
class Config:
    port: int = field(metadata={"unit": "tcp"})

print(Config.__dataclass_fields__['port'].metadata)


@dataclass
class IMUConfig:
    rate: Annotated[int, "Hz", "sampling_rate"] = field(default=100)


hints = get_type_hints(IMUConfig, include_extras=True)
print("annotated hints:", hints)
print("annotated hints:", IMUConfig.__annotations__)

# print(hints['rate'].__metadata__)  # Output: ('Hz', 'sampling_rate')


