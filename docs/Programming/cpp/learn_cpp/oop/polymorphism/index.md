---
title: C++ Polymorphism
tags:
    - cpp
    - oop
    - polymorphism
    - virtual
---

# C++ polymorphism

<div class="grid-container">
    <div class="grid-item">
        <a href="function_override">
        <p>Function override</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="virtual_destructor">
        <p>Virtual destructor</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="abstract_base_class">
        <p>Abstract base class</p>
        </a>
    </div>
</div>

Polymorphism means **one interface, many implementations**.

In C++, this usually means calling a function through a base class pointer or
reference, while the real object decides which function runs.

Example idea:

```text
Animal reference
    -> Dog object calls Dog::speak()
    -> Cat object calls Cat::speak()
```

The caller only knows that it has an `Animal`.

The actual behavior depends on whether the object is a `Dog`, `Cat`, or another
derived class.

---

## Simple example

Use `virtual` in the base class and `override` in the derived class.

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "some animal sound\n";
    }
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

`make_sound()` receives an `Animal&`, but it still calls the correct `speak()`
function for the real object.

That is runtime polymorphism.

---

## Why `virtual` matters

Without `virtual`, C++ chooses the function based on the reference or pointer
type.

With `virtual`, C++ chooses the function based on the real object type.

```cpp
const Animal& animal = dog;
animal.speak(); // calls Dog::speak() because speak() is virtual
```

Use `override` in derived classes so the compiler checks that you really are
overriding a virtual function.

---

## Key point

Polymorphism lets you write code that works with a general type, like `Animal`,
while still getting specialized behavior from derived types, like `Dog` and
`Cat`.
