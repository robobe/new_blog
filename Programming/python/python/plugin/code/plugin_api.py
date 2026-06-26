# plugin_api.py
from typing import Protocol, runtime_checkable, Any


@runtime_checkable
class Plugin(Protocol):
    name: str

    def setup(self, config: dict[str, Any]) -> None:
        ...

    def process(self, data: Any) -> Any:
        ...