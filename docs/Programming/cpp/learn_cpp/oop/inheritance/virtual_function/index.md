---
title: Cpp virtual functions
tags:
    - cpp
    - oop
    - inheritance
    - virtual
    - polymorphism
---

# C++ virtual functions

Virtual functions allow C++ to choose the function implementation using the real
object type at runtime.

This is called runtime polymorphism.

```text
Animal* animal = new Dog();

animal points to Animal interface
real object is Dog
```

Without `virtual`, C++ uses the pointer/reference type.

With `virtual`, C++ uses the real object type.

---

## The problem

Suppose we have a base class and a derived class.

```cpp
#include <iostream>

class Animal {
public:
    void speak() const
    {
        std::cout << "animal sound\n";
    }
};

class Dog : public Animal {
public:
    void speak() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    Dog dog;

    dog.speak(); // woof

    Animal& animal = dog;
    animal.speak(); // animal sound
}
```

Output:

```text
woof
animal sound
```

Why does the second call print `animal sound`?

Because `animal` is an `Animal&`, and `speak()` is not virtual.

So C++ calls `Animal::speak()`.

---

## Add virtual

Add `virtual` to the base class function.

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }
};

class Dog : public Animal {
public:
    void speak() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    Dog dog;

    Animal& animal = dog;
    animal.speak(); // woof
}
```

Output:

```text
woof
```

Now C++ checks the real object type.

The reference type is `Animal&`, but the real object is `Dog`.

So C++ calls `Dog::speak()`.

---

## Use override

Use `override` in the derived class.

```cpp
class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }
};

class Dog : public Animal {
public:
    void speak() const override
    {
        std::cout << "woof\n";
    }
};
```

`override` tells the compiler:

> this function must override a virtual function from the base class.

This protects you from mistakes.

Example mistake:

```cpp
class Dog : public Animal {
public:
    void speak() override // error if Animal::speak() is const
    {
        std::cout << "woof\n";
    }
};
```

If the base function is:

```cpp
virtual void speak() const;
```

then the derived function must also be:

```cpp
void speak() const override;
```

---

## Base pointer to derived object

Virtual functions are most useful with base pointers or base references.

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }
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
    Dog dog;

    Animal* animal = &dog;
    animal->speak(); // woof
}
```

The pointer type is `Animal*`.

The real object is `Dog`.

Because `speak()` is virtual, `Dog::speak()` is called.

---

## Heap allocation

You can also allocate the derived object on the heap and store it in a base
pointer.

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }

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
    Animal* animal = new Dog();

    animal->speak(); // woof

    delete animal;
}
```

This line creates a `Dog`, but stores it behind an `Animal*`.

```cpp
Animal* animal = new Dog();
```

Because `speak()` is virtual, calling:

```cpp
animal->speak();
```

calls `Dog::speak()`.

---

## Virtual destructor

If you delete a derived object through a base pointer, the base class should have
a virtual destructor.

```cpp
class Animal {
public:
    virtual ~Animal() = default;
};
```

Why?

```cpp
Animal* animal = new Dog();
delete animal;
```

The pointer type is `Animal*`, but the real object is `Dog`.

A virtual destructor makes sure the correct destructor chain runs.

Simple rule:

> If a class has virtual functions, give it a virtual destructor.

---

## Smart pointer example

Prefer smart pointers over raw `new` and `delete`.

```cpp
#include <iostream>
#include <memory>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }

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

    animal->speak(); // woof
}
```

`std::unique_ptr<Animal>` owns the object.

The real object is `Dog`.

When `animal` goes out of scope, the object is deleted automatically.

Because `Animal` has a virtual destructor, deletion is correct.

---

## Quick rules

| Situation | Use |
| --------- | --- |
| Function should behave differently for derived classes | `virtual` |
| Derived function overrides base virtual function | `override` |
| Delete derived object through base pointer | virtual destructor |
| Own polymorphic object | `std::unique_ptr<Base>` |
| Want runtime polymorphism | base pointer or base reference |

---

## What is not covered here

This post does not cover abstract classes or interfaces.

Those use pure virtual functions:

```cpp
virtual void speak() const = 0;
```

That should be learned in a separate post.

