from typing import Callable, TypeVar, Generic
import logging

T = TypeVar("T")
logger = logging.getLogger(__name__)

class Event(Generic[T]):
    def __init__(self) -> None:
        self._handlers: list[Callable[[T], None]] = []

    def __iadd__(self, handler: Callable[[T], None]) -> "Event[T]":
        if handler not in self._handlers:
            self._handlers.append(handler)
        return self

    def __isub__(self, handler: Callable[[T], None]) -> "Event[T]":
        try:
            self._handlers.remove(handler)
        except ValueError:
            pass
        return self

    def fire(self, item: T) -> None:
        for handler in list(self._handlers):  # copy for safety
            try:
                handler(item)
            except Exception:
                logger.exception("Event handler raised an exception")

def on_str(s: str) -> None:
    print("received:", s)

event = Event[str]()
event += on_str
event.fire("hello")

event -= on_str
event.fire("world")  # nothing happens
