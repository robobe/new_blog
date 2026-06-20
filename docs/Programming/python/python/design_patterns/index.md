---
title: Python design patterns
tags:
    - singleton
    - factory
---

<div class="grid-container">
    <div class="grid-item">
        <a href="#solid">
        <p>SOLID</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#gof">
        <p>GoF</p>
        </a>
    </div>
</div>


## SOLID
SOLID is not a design pattern. It is a set of 5 object-oriented design principles that help make code easier to maintain, extend, test, and understand.

| Principle | Name                            | Brief Description                                                |
| --------- | ------------------------------- | ---------------------------------------------------------------- |
| S         | Single Responsibility Principle | A class should have only one reason to change.                   |
| O         | Open/Closed Principle           | Open for extension, closed for modification.                     |
| L         | Liskov Substitution Principle   | Derived types must be usable wherever the base type is expected. |
| I         | Interface Segregation Principle | Prefer many small interfaces over one large interface.           |
| D         | Dependency Inversion Principle  | Depend on abstractions, not concrete implementations.            |

### Single Responsibility Principle

A class should have one job. Split unrelated responsibilities into separate classes.

Bad:

```python
class Report:
    def __init__(self, data):
        self.data = data

    def calculate_total(self):
        return sum(self.data)

    def save_to_file(self, path):
        with open(path, "w") as file:
            file.write(str(self.calculate_total()))
```

Fixed:

```python
class Report:
    def __init__(self, data):
        self.data = data

    def calculate_total(self):
        return sum(self.data)


class ReportFileWriter:
    def save(self, report, path):
        with open(path, "w") as file:
            file.write(str(report.calculate_total()))
```

### Open/Closed Principle

Code should allow new behavior without editing existing tested logic.

Bad:

```python
class Discount:
    def calculate(self, customer_type, price):
        if customer_type == "regular":
            return price * 0.95
        if customer_type == "vip":
            return price * 0.80
        return price
```

Fixed:

```python
class Discount:
    def calculate(self, price):
        return price


class RegularDiscount(Discount):
    def calculate(self, price):
        return price * 0.95


class VipDiscount(Discount):
    def calculate(self, price):
        return price * 0.80


def checkout(price, discount):
    return discount.calculate(price)
```

The bad example puts every discount rule inside one method. Adding a new customer
type means editing `Discount.calculate`, which makes old, already-tested behavior
easy to break while adding new behavior.

The fixed version keeps `checkout` and the existing discount classes closed to
changes. New behavior is added by creating another class, such as
`StudentDiscount`, that implements `calculate`. The code is open for extension
because new discount types can be introduced without rewriting the shared
checkout flow or changing the existing discount rules.


Using a Python protocol, the shared abstraction can be expressed as behavior
instead of inheritance:

```python
from typing import Protocol


class Discount(Protocol):
    def calculate(self, price: float) -> float:
        ...


class RegularDiscount:
    def calculate(self, price: float) -> float:
        return price * 0.95


class VipDiscount:
    def calculate(self, price: float) -> float:
        return price * 0.80


def checkout(price: float, discount: Discount) -> float:
    return discount.calculate(price)
```

`RegularDiscount` and `VipDiscount` do not need to inherit from `Discount`.
They match the protocol because they provide a compatible `calculate` method.
This keeps the code open to new discount implementations while allowing static
type checkers to verify that `checkout` receives an object with the required
behavior.


### Liskov Substitution Principle

A subclass should work anywhere its parent class is expected.

Bad:

```python
class Bird:
    def fly(self):
        return "Flying"


class Penguin(Bird):
    def fly(self):
        raise NotImplementedError("Penguins cannot fly")
```

Fixed:

```python
class Bird:
    pass


class FlyingBird(Bird):
    def fly(self):
        return "Flying"


class Sparrow(FlyingBird):
    pass


class Penguin(Bird):
    pass
```

This principle matters when other code receives a `Bird` and assumes every bird
can fly. For example, a migration tracker, game engine, or delivery simulation
might call `bird.fly()` on every object in a list of birds. If `Penguin`
inherits from `Bird` but raises an error when `fly` is called, the subclass
breaks code that correctly worked with the parent type.

The fixed design avoids lying in the inheritance hierarchy. Code that needs
flying behavior depends on `FlyingBird`, while code that only needs general bird
data can still use `Bird`. This keeps functions easier to reason about because
they do not need special checks like `if not isinstance(bird, Penguin)` before
calling methods. When subclasses keep the promises made by their parent classes,
new types can be added with fewer hidden runtime surprises.


### Interface Segregation Principle

Clients should not be forced to implement methods they do not use.

Bad:

```python
class Worker:
    def work(self):
        raise NotImplementedError

    def eat(self):
        raise NotImplementedError


class Robot(Worker):
    def work(self):
        return "Working"

    def eat(self):
        raise NotImplementedError("Robots do not eat")
```

Fixed:

```python
class Workable:
    def work(self):
        raise NotImplementedError


class Eatable:
    def eat(self):
        raise NotImplementedError


class Human(Workable, Eatable):
    def work(self):
        return "Working"

    def eat(self):
        return "Eating"


class Robot(Workable):
    def work(self):
        return "Working"
```

Using protocols, the small interfaces describe required behavior without forcing
classes to inherit from them:

```python
from typing import Protocol


class Workable(Protocol):
    def work(self) -> str:
        ...


class Eatable(Protocol):
    def eat(self) -> str:
        ...


class Human:
    def work(self) -> str:
        return "Working"

    def eat(self) -> str:
        return "Eating"


class Robot:
    def work(self) -> str:
        return "Working"


def start_shift(worker: Workable) -> str:
    return worker.work()


def lunch_break(worker: Eatable) -> str:
    return worker.eat()
```

`Human` satisfies both protocols because it can work and eat. `Robot` satisfies
only `Workable`, so code that needs eating behavior cannot accidentally receive
a robot when checked by a static type checker.


### Dependency Inversion Principle

High-level code should depend on abstractions instead of concrete classes.

Bad:

```python
class MySQLDatabase:
    def save(self, data):
        print(f"Saving {data} to MySQL")


class UserService:
    def __init__(self):
        self.database = MySQLDatabase()

    def create_user(self, user):
        self.database.save(user)
```

Fixed:

```python
class Database:
    def save(self, data):
        raise NotImplementedError


class MySQLDatabase(Database):
    def save(self, data):
        print(f"Saving {data} to MySQL")


class UserService:
    def __init__(self, database):
        self.database = database

    def create_user(self, user):
        self.database.save(user)
```

Using a protocol:

```python
from typing import Protocol


class Database(Protocol):
    def save(self, data):
        ...


class MySQLDatabase:
    def save(self, data):
        print(f"Saving {data} to MySQL")


class UserService:
    def __init__(self, database: Database):
        self.database = database

    def create_user(self, user):
        self.database.save(user)
```

---


## GoF
## Singleton

Singleton is a design pattern where a class can have only one instance, and that instance is globally accessible.

```python title="simple implementation"
class Singleton:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

a = Singleton()
b = Singleton()

print(a is b)  # True
```

!!! tip ""
    There a lot of design implementation
    The above is the simple one


---

## Factory
The Factory Pattern is a design pattern used to create objects without exposing the exact class being created.

```python
class Animal:
    def say_hello(self):
        raise NotImplementedError

class Dog(Animal):
    def say_hello(self):
        return "Hello from Dog!"

class Cat(Animal):
    def say_hello(self):
        return "Hello from Cat!"

def animal_factory(kind):
    if kind == "dog":
        return Dog()
    elif kind == "cat":
        return Cat()
    else:
        raise ValueError("Unknown animal")

animal = animal_factory("dog")
print(animal.say_hello())
```

---

## Observer

The Observer Pattern is a design pattern where one object (the subject) notifies other objects (observers) when its state changes.

Core idea:

- One object publishes updates
- Many objects subscribe to receive updates

When the subject changes, all observers are automatically notified.
    
```python
class Subject:
    def __init__(self):
        self.observers = []

    def subscribe(self, observer):
        self.observers.append(observer)

    def notify(self, message):
        for observer in self.observers:
            observer.update(message)

class Observer:
    def update(self, message):
        print("Received:", message)

subject = Subject()
observer1 = Observer()
observer2 = Observer()

subject.subscribe(observer1)
subject.subscribe(observer2)
subject.notify("Hello Observers!")
```

---

## command

The Command Pattern is a design pattern that encapsulates a request as an object so it can be stored, passed, or executed later.

Instead of calling a method directly, you create a command object that represents the action.

```python
```

---

## Adapter
The adapter converts one interface into another interface that the client expects.

It allows incompatible classes to work together.

```python
class OldAPI:
    def specific_request(self):
        return "Old API response"
    
# the client call requires a different interface
# we want to use OldAPI but it doesn't match the expected interface
# the Adapter allows us to use OldAPI with the new interface
# the client code can call Adapter.request() and it will internally call OldAPI.specific_request()

class Adapter:
    def __init__(self, old_api):
        self.old_api = old_api

    def request(self):
        return self.old_api.specific_request()
    
# client code
old_api = OldAPI()
adapter = Adapter(old_api)

print(adapter.request())
```

---

## decorator
Decorator lets you add behavior to an object dynamically without modifying the original class.
Instead of inheritance, it uses composition (wrapping an object).

```python
class Coffee:
    def cost(self):
        return 5

# decorator
class MilkDecorator:
    def __init__(self, coffee):
        self.coffee = coffee

    def cost(self):
        return self.coffee.cost() + 2

# another decorator
class SugarDecorator:
    def __init__(self, coffee):
        self.coffee = coffee

    def cost(self):
        return self.coffee.cost() + 1

# usage

coffee = Coffee()
coffee = MilkDecorator(coffee)
coffee = SugarDecorator(coffee)

print(coffee.cost())
```

!!! info "@decorator"
    python has language decorator
    it the same idea but work on method level not object level
    
---

## Facade

Provide a simple interface to a complex subsystem.

Instead of the client interacting with many classes, it interacts with one facade class that coordinates everything

```python
class imu:
    def start(self):
        print("imu started")

class camera:
    def start(self):
        print("camera started")

class motor_controller:
    def start(self):
        print("motor controller start")

# facade
class robot():
    def __init__(self):
        self.imu = imu()
        self.camera = camera()
        self.motor = motor_controller()

    def start():
        self.imu.start()
        self.camera.start()
        self.motor.start()
        
```

---

## Template Method 
Define the skeleton of an algorithm in a base class, but let subclasses implement some steps.

The algorithm structure stays fixed, while specific steps can vary.


```python
class DataProcessor:

    def run(self):
        self.load()
        self.process()
        self.save()

    def load(self):
        print("Loading data")

    def process(self):
        raise NotImplementedError()

    def save(self):
        print("Saving results")

class ImageProcessor(DataProcessor):

    def process(self):
        print("Processing image")

class TextProcessor(DataProcessor):

    def process(self):
        print("Processing text")

img = ImageProcessor()
img.run()

print("--" * 20)
txt = TextProcessor()
txt.run()
```

---

## Proxy

A Proxy is an object that acts as a substitute for another object and controls access to it.

for example RPC client is a proxy to the remote machine functionality

The proxy has the same interface as the real object but can:

- delay creation (lazy loading)
- control access (security proxy)
- cache results (cache proxy)
- handle remote communication (remote proxy)

---

## Iterator
Sequentially access the elements of a collection without exposing its underlying representation.

**So the client does not need to know how the collection is implemented.**
The iterator **moves** through the collection.

```python

class DataIterator:
    def __init__(self, data):
        self.data = data
        self.index = 0

    def __next__(self):
        if self.index >= len(self.data):
            raise StopIteration
        value = self.data[self.index]
        self.index += 1
        return value
    
class DataCollection:
    def __init__(self):
        self.data = list(range(1, 11))

    def __iter__(self):
        return DataIterator(self.data)
    
data = DataCollection()

iterator = iter(data)

print(next(iterator))
print(next(iterator))
print(next(iterator))
```


---

## Reference
- [Understanding The Gang of Four (GOF) design patterns using Python — Part1](https://learncsdesigns.medium.com/understanding-the-gang-of-four-gof-design-patterns-using-python-54fcf2086d44)
