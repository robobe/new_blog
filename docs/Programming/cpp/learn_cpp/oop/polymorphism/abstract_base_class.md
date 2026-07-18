---
title: C++ Abstract Base Class
tags:
    - cpp
    - oop
    - polymorphism
    - abstract
    - interface
---

# C++ abstract base class

An abstract base class is a class that cannot be used to create objects directly.

It is used as a common interface for derived classes.

In C++, a class becomes abstract when it has at least one pure virtual function.

```cpp
virtual void speak() const = 0;
```

The `= 0` means:

> derived classes must implement this function.

---

## Simple example

```cpp
#include <iostream>

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

Output:

```text
woof
meow
```

`Animal` defines the interface.

`Dog` and `Cat` provide the real behavior.

---

## You cannot create the base object

This is not allowed:

```cpp
Animal animal; // error: Animal is abstract
```

Why?

Because `Animal::speak()` has no implementation.

Only derived classes that implement all pure virtual functions can be created.

---

## Practical use

Abstract base classes are useful when different classes should be used through
the same interface.

Example: different file exporters.

```cpp
#include <iostream>
#include <string>

class Exporter {
public:
    virtual void export_data(const std::string& data) const = 0;
    virtual ~Exporter() = default;
};

class JsonExporter : public Exporter {
public:
    void export_data(const std::string& data) const override
    {
        std::cout << "{ \"data\": \"" << data << "\" }\n";
    }
};

class CsvExporter : public Exporter {
public:
    void export_data(const std::string& data) const override
    {
        std::cout << "data\n" << data << "\n";
    }
};

void save_report(const Exporter& exporter)
{
    exporter.export_data("robot status ok");
}

int main()
{
    JsonExporter json;
    CsvExporter csv;

    save_report(json);
    save_report(csv);
}
```

`save_report()` does not care which exporter it receives.

It only needs something that follows the `Exporter` interface.

This lets you add another exporter later without changing `save_report()`.

---

## Key point

Use an abstract base class when you want to define what derived classes must do,
without deciding how they do it.

Common rules:

- use pure virtual functions for required behavior
- use `override` in derived classes
- use a virtual destructor in the base class
