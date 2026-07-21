---
title: C++ dynamic_cast
tags:
    - cpp
    - oop
    - polymorphism
    - dynamic_cast
---

# C++ `dynamic_cast`

`dynamic_cast` is a C++ cast used with polymorphic classes.

It checks the real object type at runtime before converting a base class pointer
or reference to a derived class type.

Use it when you have a pointer or reference to a base class, but sometimes need
to access behavior that exists only in one derived class.

---

## Why it exists

In object-oriented C++, code often works through a base class:

```cpp
Animal* animal;
```

At runtime, that pointer may actually point to a `Dog`, `Cat`, or another class
derived from `Animal`.

Virtual functions are the preferred way to use polymorphism. But sometimes you
need to ask:

> Is this `Animal` really a `Dog`?

That is where `dynamic_cast` is useful.

---

## Simple example

```cpp
#include <iostream>
#include <vector>

class Animal {
public:
    virtual ~Animal() = default;
};

class Dog : public Animal {
public:
    void fetch() const
    {
        std::cout << "Dog fetches the ball\n";
    }
};

class Cat : public Animal {
public:
    void scratch() const
    {
        std::cout << "Cat scratches the sofa\n";
    }
};

int main()
{
    Dog dog;
    Cat cat;

    std::vector<Animal*> animals = {&dog, &cat};

    for (Animal* animal : animals) {
        Dog* dog_ptr = dynamic_cast<Dog*>(animal);

        if (dog_ptr != nullptr) {
            dog_ptr->fetch();
        } else {
            std::cout << "This animal is not a Dog\n";
        }
    }
}
```

Output:

```text
Dog fetches the ball
This animal is not a Dog
```

- The first `Animal*` points to a `Dog`, so `dynamic_cast<Dog*>` succeeds.

- The second `Animal*` points to a `Cat`, so the cast fails and returns `nullptr`.

---

## Important rule

`dynamic_cast` works for downcasting only when the base class is polymorphic.

That means the base class must have at least one `virtual` function.

Commonly, the base class has a virtual destructor:

```cpp
class Animal {
public:
    virtual ~Animal() = default;
};
```

This makes the class polymorphic and also allows safe deletion through a base
class pointer.

---

## Pointer vs reference

### casting a pointer:

```cpp
Dog* dog = dynamic_cast<Dog*>(animal);
```

If the object is not a `Dog`, the result is `nullptr`.

### casting a reference:

```cpp
Dog& dog = dynamic_cast<Dog&>(animal);
```

If the object is not a `Dog`, C++ throws `std::bad_cast`.

For beginner code, pointer casts are usually easier because checking for
`nullptr` is simple.

---

## When to use `dynamic_cast`

Use `dynamic_cast` when:

- You have a base class pointer or reference.
- You need to know whether the real object is a specific derived type.
- You need to call a function that exists only on that derived type.
- The type check is part of the program logic.

Example:

```cpp
Animal* animal = get_animal();

if (Dog* dog = dynamic_cast<Dog*>(animal)) {
    dog->fetch();
}
```

This means:

- Try to treat `animal` as a `Dog`.
- If it really is a `Dog`, call `fetch()`.
- If it is not a `Dog`, skip the block.

---

## When not to use it

If you need many `dynamic_cast` checks, the class design may be wrong.

This is usually better:

```cpp
animal->speak();
```

Than this:

```cpp
if (Dog* dog = dynamic_cast<Dog*>(animal)) {
    dog->speak();
} else if (Cat* cat = dynamic_cast<Cat*>(animal)) {
    cat->speak();
}
```

Prefer virtual functions when all derived classes should support the same
operation.

Use `dynamic_cast` only when you really need behavior that belongs to a specific
derived type.

---

## Key points

- `dynamic_cast` performs a runtime type check.
- It is mainly used for safe downcasting in inheritance hierarchies.
- The base class must be polymorphic.
- Pointer casts return `nullptr` when they fail.
- Reference casts throw `std::bad_cast` when they fail.
- Prefer virtual functions for normal polymorphic behavior.
