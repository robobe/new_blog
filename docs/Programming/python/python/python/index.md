---
title: Python 
tags:
    - __slots__
    - contextmanager
    - iterator
    - generator
    - iterable
    - closure
    - descriptor
---

<div class="grid-container">
    <div class="grid-item">
        <a href="#slots">
            <p>__slots__</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#context-manager">
            <p>context manager</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#iterator-and-generator">
            <p>Iterator and Generator</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#closure">
            <p>Closures (one step to Decorator)</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#descriptor">
            <p>Descriptor</p>
        </a>
    </div>
</div>

## __slots__

In Python, `__slots__` is a **class attribute** used to **restrict** the attributes that instances of the class can have and to save memory.

Normally, every Python object stores its attributes in a dictionary (`__dict__`), which is flexible but consumes more memory.

`__slots__` replaces that dictionary with a fixed structure, making objects smaller and faster to access.

```python title="normal class"
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

p = Point(1, 2)

print(p.__dict__)
```

```python title="__slot__ class"
class Point:
    __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.x = x
        self.y = y

p = Point(1, 2)
print(p.x, p.y)
```

!!! info "dynamic programming"
    Python uses a dictionary (`__dict__`) by default instead of `__slots__` mainly because Python is designed to be dynamic and flexible. A dictionary allows objects to change structure at runtime, which is a core philosophy of Python.
    

### Demo: memory efficient

<details>
<summary>Code</summary>
```
--8<-- "docs/Programming/python/python/python/code/slots_memory_demo.py"
```
</details>

---

### Summary 

| Feature                          | `__dict__` (Default Python objects)   | `__slots__`                                           |
| -------------------------------- | ------------------------------------- | ----------------------------------------------------- |
| **Attribute storage**            | Stored in a dynamic dictionary        | Stored in fixed slots inside the object               |
| **Memory usage**                 | Higher (each object has a dictionary) | Lower (no dictionary per instance)                    |
| **Dynamic attributes**           | Allowed (`obj.new_attr = 5`)          | Not allowed unless `"__dict__"` is added              |
| **Attribute access speed**       | Slightly slower (dictionary lookup)   | Slightly faster (direct slot access)                  |
| **Flexibility**                  | Very flexible                         | Rigid structure                                       |
| **Inheritance complexity**       | Simple                                | More complex (subclasses must define slots carefully) |
| **Metaprogramming support**      | Excellent                             | Limited                                               |
| **Debugging / introspection**    | Easy (`obj.__dict__`)                 | Harder (no dictionary)                                |
| **Memory benefit visible when**  | Few objects → difference negligible   | Many objects (thousands or millions)                  |
| **Typical use cases**            | General Python programming            | High-performance data objects                         |
| **Compatibility with libraries** | Fully compatible                      | Some libraries expect `__dict__`                      |
| **Weak references**              | Supported automatically               | Need `"__weakref__"` in slots                         |
| **Default behavior**             | Yes                                   | No (must be explicitly defined)                       |


!!! tip "hybrid mode"
    We can mix `__slots__` with `__dict__` 

    ```python
    class Point:
    __slots__ = ("x", "y", "__dict__")
    ```
    
### Practical rules

| Situation                           | Recommendation                  |
| ----------------------------------- | ------------------------------- |
| Normal application code             | Use default (`__dict__`)        |
| Many lightweight objects (millions) | Use `__slots__`                 |
| Data containers / sensor messages   | `__slots__` is often beneficial |
| Dynamic attributes needed           | Use `__dict__`                  |


---

## context manager

A context manager in Python is an object that manages setup and cleanup automatically around a block of code using the with statement.

It ensures that resources are properly released, even if an error occurs.

```
setup → use → cleanup
```

```python
with open("data.txt") as f:
    content = f.read()
```

- file is opend
- code inside `with` run
- file is closed 

```python
f = open("data.txt")
try:
    content = f.read()
finally:
    f.close()
```

### Custom context manager

Class with implementation

- `__enter__()`
- `__exit__()`

```python
class MyContext:
    def __enter__(self):
        print("enter")
        return "resource"

    def __exit__(self, exc_type, exc, tb):
        print("exit")
```

```python title="usage"
with MyContext() as r:
    print(r)
```

#### using decorator

```python
from contextlib import contextmanager

@contextmanager
def my_context():
    print("enter")
    yield "resource"
    print("exit")
```

---

## Iterator and Generator

Generators and iterators are closely related in Python.
Both are used to **produce values one at a time (lazy evaluation)** instead of creating all values in memory at once.

A generator is a special, easy way to create an iterator.

| Feature          | Iterator                                     | Generator                                  |
| ---------------- | -------------------------------------------- | ------------------------------------------ |
| What it is       | Object that implements the iterator protocol | Special function that produces an iterator |
| How it's created | Class with `__iter__()` and `__next__()`     | Function using `yield`                     |
| Complexity       | More code                                    | Much simpler                               |
| Memory usage     | Lazy (one value at a time)                   | Lazy (same benefit)                        |
| Typical usage    | Custom iteration logic                       | Streaming sequences                        |
| State handling   | Must store state manually                    | Python keeps state automatically           |

### Iterator

An iterator is any object that implements:

- `__iter__()`
- `__next__()`

```python
class Counter:
    def __init__(self, max_value):
        self.max = max_value
        # state
        self.current = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.current >= self.max:
            raise StopIteration

        value = self.current
        self.current += 1
        return value


c = Counter(3)

for i in c:
    print(i)
```

- Iteration state `self.current`
- Stopping condition `raise StopIteration`
- Value generation 

---

### Iterable
An iterable is an object that can produce an iterator.

An object is iterable if it implements:

```python
__iter__()
```

which return an **iterator**

```python
nums = [1,2,3]

it = iter(nums)   # iterator
next(it)
next(it)
```

---

### Generator
A generator is a simpler way to create an iterator using yield.

```python
def counter(max_value):
    current = 0
    while current < max_value:
        yield current
        current += 1


for i in counter(3):
    print(i)
```

Python automatically handles:

- iterator creation
- state saving
- stopping iteration


!!! tip "yield"
    yield pauses the function and remembers its state.

    ```
    call generator
          ↓
    yield value
          ↓
    pause function
          ↓
    resume on next iteration
    ```
    
#### Generator Expression
Short syntax similar to list comprehension

```python title="list"
[x*x for x in range(10)]
```

```python title="generator"
(x*x for x in range(10))
```

| Type      | Memory             |
| --------- | ------------------ |
| list      | stores everything  |
| generator | computes on demand |

```python
>>> g = (x*x for x in range(5))
>>> print(type(g))
<class 'generator'>
>>> next(g)
0
>>> next(g)
1
>>> next(g)
4
>>> next(g)
9
```

#### Generator pipeline

- `send()`
- `close()`
A generator pipeline is a chain of generators where data flows lazily through multiple processing steps, making code efficient, modular, and memory-friendly.

```
data → generator1 → generator2 → generator3 → result
```

<details>
<summary>pipeline demo</summary>
```
--8<-- "docs/Programming/python/python/python/code/gen_pipeline_demo.py"
```
</details>

##### send

```python
def accumulator():
    total = 0
    while True:
        value = yield total
        total += value

acc = accumulator()
print(next(acc))  # Start the generator, prints 0
print(acc.send(5))  # Send 5, prints 5
print(acc.send(10))  # Send 10, prints 15
```

##### close

```python
def my_generator():
    try:
        while True:
            yield "running"
    finally:
        print("cleanup on close")


gen = my_generator()
print(next(gen))  # "running"
print(next(gen))  # "running"
gen.close()  # This will trigger the cleanup code in the generator
```

---

## Closure
A closure is a function that captures and keeps access to variables from its enclosing scope.

```python
def outer():
    x = 10

    def inner():
        print(x)

    return inner


f = outer()
f()
```

---

### How it work
python save reference to the outer variable in `__closure__' attribute
it's tuple that it's item point to variable

```python
print(func.__closure__)
# outer variable data
print(func.__closure__[0].cell_contents)
```

---

## Descriptor
[Descriptor Deep Dive](https://youtu.be/7SUzTOkUVLY)

A descriptor is any object that implements one or more of these methods:

- `__get__(self, instance, owner)`
- `__set__(self, instance, value)`
- `__delete__(self, instance)`


### Demo:

<details>
<summary>Descriptor</summary>
```
--8<-- "docs/Programming/python/python/python/code/descriptor_demo.py"
```
</details>

- instance → the object (obj in the example)
- owner → the class (MyClass in the example)

### Descriptor type

- Data descriptor
- Non-Data descriptor

#### Data descriptor
Has `__get__` + `__set__` (or `__delete__`)

!!! info 
    👉 Takes priority over instance attributes


#### Non-Data descriptor
Has only `__get__`

!!! info
    👉 Can be overridden by instance attributes

<details>
<summary>Non-data descriptor demo</summary>
```
--8<-- "docs/Programming/python/python/python/code/descriptor_non_data.py"
```
</details>