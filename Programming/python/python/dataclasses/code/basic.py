from dataclasses import dataclass,asdict, astuple

@dataclass
class Point:
    x: float
    y: float

p = Point(1.0, 2.0)
print(asdict(p))
print(astuple(p))