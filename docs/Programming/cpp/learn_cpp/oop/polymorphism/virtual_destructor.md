---
title: C++ Virtual Destructor
tags:
    - cpp
    - oop
    - polymorphism
    - destructor
    - virtual
---

# C++ virtual destructor

A virtual destructor is needed when you delete a derived object through a base
class pointer.

If a class is meant to be used polymorphically, its destructor should usually be
virtual.

Simple rule:

> If a class has virtual functions, give it a virtual destructor.

---

## The problem

This code uses a base pointer to hold a derived object:

```cpp
Animal* animal = new Dog();
delete animal;
```

The pointer type is `Animal*`.

The real object is `Dog`.

When `delete animal` runs, C++ must destroy the full `Dog` object, not only the
`Animal` part.

---

## Simple example

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }

    virtual ~Animal()
    {
        std::cout << "Animal destructor\n";
    }
};

class Dog : public Animal {
public:
    void speak() const override
    {
        std::cout << "woof\n";
    }

    ~Dog() override
    {
        std::cout << "Dog destructor\n";
    }
};

int main()
{
    Animal* animal = new Dog();

    animal->speak();

    delete animal;
}
```

Output:

```text
woof
Dog destructor
Animal destructor
```

Because `Animal` has a virtual destructor, deleting through `Animal*` correctly
runs both destructors.

---

## Common modern usage

Prefer smart pointers instead of raw `new` and `delete`.

```cpp
#include <iostream>
#include <memory>

class Animal {
public:
    virtual void speak() const = 0;
    virtual ~Animal() = default;
};

class Dog : public Animal {
public:
    void speak() const override
    {
        std::cout << "woof\n";
    }
};

int main()
{
    std::unique_ptr<Animal> animal = std::make_unique<Dog>();
    animal->speak();
}
```

`std::unique_ptr<Animal>` owns the object.

When the pointer goes out of scope, it deletes the object automatically.

The base destructor still must be virtual so the derived object is destroyed
correctly.

---

## Key point

Use a virtual destructor in a base class when:

- the class has virtual functions
- derived objects may be deleted through a base pointer
- the base class is used as a polymorphic interface
