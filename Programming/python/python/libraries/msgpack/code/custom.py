import msgpack
from dataclasses import dataclass
from typing import Any, ClassVar

@dataclass
class Pose:
    x: float
    y: float
    z: float

    TYPE_TAG: ClassVar[str] = "Pose"
    VERSION: ClassVar[int] = 1

    def to_msgpack(self) -> list:
        # msgpack-friendly representation
        return [self.TYPE_TAG, self.VERSION, self.x, self.y, self.z]

    @classmethod
    def from_msgpack(cls, obj: Any) -> "Pose":
        # decode from msgpack-friendly representation
        if not (isinstance(obj, list) and len(obj) == 5 and obj[0] == cls.TYPE_TAG):
            raise TypeError(f"Not a {cls.TYPE_TAG} msgpack object: {obj!r}")

        version = obj[1]
        if version != cls.VERSION:
            raise ValueError(f"Unsupported {cls.TYPE_TAG} version: {version}")

        x, y, z = obj[2], obj[3], obj[4]
        return cls(float(x), float(y), float(z))

    @staticmethod
    def msgpack_default(obj: Any) -> Any:
        # hook for msgpack.packb(default=...)
        if isinstance(obj, Pose):
            return obj.to_msgpack()
        raise TypeError(f"Cannot serialize type: {type(obj)!r}")

    @staticmethod
    def decode_any(obj: Any) -> Any:
        # safe helper: only converts if it matches, otherwise returns as-is
        if isinstance(obj, list) and len(obj) == 5 and obj[0] == Pose.TYPE_TAG:
            return Pose.from_msgpack(obj)
        return obj


# ---- usage ----
pose = Pose(1.0, 2.0, 3.0)

packed = msgpack.packb(pose, default=Pose.msgpack_default, use_bin_type=True)
raw = msgpack.unpackb(packed, raw=False)

pose_obj = Pose.decode_any(raw)

print(pose_obj)
print(type(pose_obj))