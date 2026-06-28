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

## Protocol vs Inheritances

Python Protocols and inheritance are two different ways to define what an object can do.

The short version is:

- **Inheritance** says: "This class IS a type of another class."
- **Protocol** says: "This class HAS the required methods."

A protocol checks behavior, not inheritance.
In the above example any class with a `speak()` method satisfies the protocol.

- Use **inheritance** when you want to share implementation (common logging, lifecycle methods, utility functions).
- Use **Protocols** when you want to define interchangeable components based on what they can do, not what they inherit from.

---

### @runtime_checkable
@runtime_checkable makes a protocol usable with isinstance() or issubclass() at runtime.

```python
--8<-- "docs/Programming/python/python/protocol/code/without_decorate.py"
```


```python
--8<-- "docs/Programming/python/python/protocol/code/using_runtime_checkable.py"
```

---

## Once class, many protocol 

```python
from typing import Protocol

class Startable(Protocol):
    def start(self):
        ...

class Stoppable(Protocol):
    def stop(self):
        ...

class Configurable(Protocol):
    def configure(self, cfg):
        ...
```

A camera implements all three.

```python
class Camera:

    def start(self):
        ...

    def stop(self):
        ...

    def configure(self, cfg):
        ...
```

Now the same object can be used in different places.

```python
def boot(device: Startable):
    device.start()

def shutdown(device: Stoppable):
    device.stop()

def setup(device: Configurable):
    device.configure(...)
```