---
title: Python type annotation
tags:
    - python
    - typing
    - annotation
    - pyright
    - vscode
---

### primitive

```python
var: str = "hello"
b: bool = False
f: float = 1.1
n: int = 1
nb: int = True

def foo(names: str) -> None
    pass
```

```python title="union"
var: str | int = 10
var = "hello"

var: int | None = None

from typing import Optional

var: Optional(int) = None
```

### collections

```python title="list"
numbers: list[int] = [1, 2, 3]
```

```python title="tuple"
data: tuple[int, str, bool] = (1, "hello", True)
t_of_int[int, ...] = (1, 2, 3, 4, 5, 6)
```

```python title="sets"
my_set: set[int] = {1, 2, 3}
```

```python title="dict"
my_dict: dict[str, int] = {"a": 5}
```

### 

```python title="return value"
def get_data() -> int | None
    ...

```

!!! tip "TYPE_CHECKING"
    TYPE_CHECKING is a boolean constant:

    - **False** at runtime
    - **True** only when a type checker runs (mypy, pyright, pylance, etc.)

    ```python
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from mymodule import BigClass

    ```
    - Type checker: ✅ sees BigClass
    - Runtime: ❌ import is skipped

---



## VSCode

```json
{
  "python.analysis.typeCheckingMode": "strict"
}
```

- off
- basic
- standard
- strict

## RUFF

```ini title="pyproject.toml"
[tool.ruff]
target-version = "py312"
line-length = 88
```

---

<div class="grid-container">
    <div class="grid-item">
        <a href="#generic">
        <p>Generic</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="#callable">
        <p>callable</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="#cast">
        <p>Cast</p>
        </a>
    </div>
</div>


## generic
Generics allow you to write code that can work with **different types** while maintaining type safety. it like templates or placeholders for types that get filled in when you **actually use the code**.


### Demo: generic event class

```python
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

```

---

## callable

```
Callable[[arguments], return]
```

```python
from collections.abc import Callable

def func(callback: Callable[[int], str]) -> str:
    return callback(10)

def foo(data: int) -> str:
    return f"Hello, your number is {data}"

print(func(foo))

```

---

## cast
typing.cast` comes in. It's a function in Python's typing module that lets you tell the computer, "Hey, even though this variable looks like one thing, I want you to treat it like something else."

```python
from typing import cast

my_variable = cast(TargetType, source_variable)
```


Just remember, `typing.cast` doesn't actually change the **underlying** value of your variable – it just tells the type checker how to interpret it. So, if you try to do something that doesn't make sense for the target type, you'll still get an error at runtime.

---

## Reference
- [Why Use Types | Python Type Hints for Beginners ](https://www.youtube.com/playlist?list=PLLlTVphLQsuPCjAS5QbMf4HAedDPRPPHA)