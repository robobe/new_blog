---
title: C++ Function Override
tags:
    - cpp
    - oop
    - polymorphism
    - override
---

# C++ function override

Function overriding means a derived class provides its own version of a
`virtual` function from a base class.

The base class defines the interface.

The derived class defines the specialized behavior.

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
};

class Dog : public Animal {
public:
    void speak() const override
    {
        std::cout << "woof\n";
    }
};

void make_sound(const Animal& animal)
{
    animal.speak();
}

int main()
{
    Dog dog;
    make_sound(dog);
}
```

Output:

```text
woof
```

`make_sound()` receives an `Animal&`, but the real object is a `Dog`.

Because `speak()` is `virtual`, C++ calls `Dog::speak()`.

---

## Why use override

Use `override` when a derived class replaces a virtual function.

```cpp
void speak() const override
```

`override` tells the compiler:

> this function must override a virtual function from the base class.

This is important because the function signature must match exactly.

---

## Common mistake

The base function is `const`:

```cpp
class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal sound\n";
    }
};
```

But the derived function forgets `const`:

```cpp
class Dog : public Animal {
public:
    void speak() override // error
    {
        std::cout << "woof\n";
    }
};
```

This does not match the base function.

The correct override is:

```cpp
class Dog : public Animal {
public:
    void speak() const override
    {
        std::cout << "woof\n";
    }
};
```

`override` catches this mistake at compile time.

Without `override`, the compiler may treat the derived function as a new
function instead of an override.

---

## Virtual and override rules

Use these rules:

| Rule | Meaning |
| ---- | ------- |
| Put `virtual` in the base class | enables runtime polymorphism |
| Put `override` in the derived class | asks the compiler to check the override |
| `override` does not make a function virtual | the base function must already be virtual |
| The function signature must match | same name, parameters, `const`, and qualifiers |
| Use a virtual destructor in polymorphic base classes | allows safe deletion through a base pointer |

Example:

```cpp
class Animal {
public:
    virtual void speak() const = 0;
    virtual ~Animal() = default;
};

class Dog : public Animal {
public:
    void speak() const override
    {
    }
};
```

Wrong example:

```cpp
class Animal {
public:
    void speak() const
    {
    }
};

class Dog : public Animal {
public:
    void speak() const override // error: Animal::speak() is not virtual
    {
    }
};
```

`override` is a safety check.

It does not replace `virtual`.

---

## Key point

Always use `override` when overriding a virtual function.

It makes mistakes visible early, especially when you accidentally change:

- `const`
- function parameters
- return type
- reference qualifiers

---

## Pure virtual function

A pure virtual function has `= 0`.

It means the base class does not provide normal behavior.

Derived classes must implement the function.

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const = 0;
};

class Dog : public Animal {
public:
    void speak() const override
    {
        std::cout << "woof\n";
    }
};

class Cat : public Animal {
public:
    void speak() const override
    {
        std::cout << "meow\n";
    }
};

void make_sound(const Animal& animal)
{
    animal.speak();
}

int main()
{
    Dog dog;
    Cat cat;

    make_sound(dog);
    make_sound(cat);
}
```

**You cannot create an object directly from a class that has a pure virtual
function**:

```cpp
Animal animal; // error: Animal is abstract
```

Use a pure virtual function when the base class should define the interface, but
each derived class must define its own behavior.

---
