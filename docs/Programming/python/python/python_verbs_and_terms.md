---
title: Python verbs and terms
tags:
    - python
    - duck
---

{{ page_folder_links() }}

## Duck Typing in python

```
If it walks like a duck and it quacks like a duck, then it must be a duck.
```

In the context of programming, this means that if an object **provides** the **necessary** methods and attributes required for a certain **operation**, it can be **treated** as if it were of a specific type


---

## Monkey Patching / dynamic class modification

When we add / modify attribute/methods of class at runtime

```python
class MyClass:
    pass  # empty class

def greet(self):
    print("Hello!")

# Add behavior at runtime
MyClass.greet = greet

obj = MyClass()
obj.greet()  # Hello!

```
