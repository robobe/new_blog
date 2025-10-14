---
tags:
    - python
    - protocol
---


{{ page_folder_links() }}

A protocol defines a set of methods and properties that a class must have — **without requiring inheritance**.

!!! note "duck typing"
    If it looks like a duck, swims like a duck, and quacks like a duck… congratulations, Python will call it a duck.

The same idea is for protocol, if you can do the job i don't care how you are or in other words “Any object that looks like this, I can use it.”


```python
from typing import Protocol

class Speaker(Protocol):
    def speak(self) -> None:
        ...

class Dog:
    def speak(self):
        print("Woof!")

class Cat:
    def speak(self):
        print("Meow!")

def make_it_talk(animal: Speaker) -> None:
    animal.speak()

make_it_talk(Dog())
make_it_talk(Cat())

```

---

### @runtime_checkable
@runtime_checkable makes a protocol usable with isinstance() or issubclass() at runtime.

```python
--8<-- "code/without_decorate.py"
```


```python
--8<-- "code/using_runtime_checkable.py"
```