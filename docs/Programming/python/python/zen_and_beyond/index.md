---
title: Python ZEN and other prase
tags:
    - python
    - zen
    - duck
    - monkey
---

<div class="grid-container">
 <div class="grid-item">
        <a href="#zen">
        <p>ZEN</p>
        </a>
    </div>
     <div class="grid-item">
        <a href="#everything-is-an-object">
        <p>Everything is an object</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#duck-typing">
        <p>Duck typing</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="#mutability">
        <p>Mutability</p>
        </a>
    </div>
</div>

---
### ZEN

```python
import this
```

#### Beautiful is better than ugly
It means:
- Code expresses intent clearly
- No unnecessary boilerplate
- Uses Python idioms
- Avoids clever hacks
- Easy for another developer to read

❌ Ugly

```python
def is_adult(age):
    if age >= 18:
        return True
    else:
        return False
```

✨ Beautiful

```python
def is_adult(age):
    return age >= 18
```


---


### Mutability
Mutability = Can the object change after it is created?

| Type      | Can Change? |
| --------- | ----------- |
| Mutable   | ✅ Yes       |
| Immutable | ❌ No        |

#### Immutable

- int
- float
- str
- tuple
- bool

```python
x = 5
print(id(x))

x = x + 1
print(id(x))
```

#### Mutable
- Lists
- Dict
- Sets


##### Demo: mutable default argument trap

```python
def add_item(item, lst=[]):
    lst.append(item)
    return lst

print(add_item(1))
print(add_item(2))
```

```python title="result"
[1]
[1, 2]
```

---

!!! tip 
    - **Mutable**: object can chanf internally
    - **Immutable**: object cannot change, only replace
    
---

### Duck typing
The name comes from this idea:

```"If it walks like a duck and quacks like a duck, it’s a duck.”```

In python terms

```If an object behaves like something (has the required methods), we don’t care what its type is```


Python cares about **behavior**, not **declared** type.

---

## Everything is an object
Every value in Python carries data and behavior, and it has a type at runtime.

In Python an **object** has:
- **Identity**: where it live in memory
- **Type**: what kind of object it is
- **Value**: the data it contain

Python objects:

- Numbers
- Strings
- Functions
- Classes
- Modules
- Exceptions
- types

### Numbers

```python
x = 5

print(id(x))     # identity (memory address)
print(type(x))   # type
print(x)         # value
```

#### variable don't hold values - they holed reference

```python
a = 5
b = a

print(id(a))
print(id(b))

print(id(a) == id(b))  # True
```

#### behavior

```python

print(dir(int))
```


---

### function are an object

```python
def greet():
    print("Hello")

print(id(greet))  # This will print the memory address of the function object
print(type(greet))  # This will print <class 'function'>, indicating that greet
```

```python title="use function attribute to store data"
def counter():
    counter.count += 1
    return counter.count

# initialize attribute
counter.count = 0

print(counter())
print(counter())
print(counter())
```

---