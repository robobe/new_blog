---
title: Python design patterns
tags:
    - singleton
    - factory
---

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

## Reference
- [Understanding The Gang of Four (GOF) design patterns using Python — Part1](https://learncsdesigns.medium.com/understanding-the-gang-of-four-gof-design-patterns-using-python-54fcf2086d44)