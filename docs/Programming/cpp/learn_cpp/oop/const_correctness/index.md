---
title: Cpp const correctness
tags:
    - cpp
    - oop
    - const
    - encapsulation
---

# C++ const correctness in OOP

`const` helps separate functions that only read an object from functions that
modify an object.

In OOP, this belongs mostly to encapsulation.

The class should make it clear:

- which functions change object state
- which functions only read object state

---

## Const member function

`const` after a member function means:

> this function promises not to modify the object.

```cpp
#include <iostream>

class Dog {
public:
    void set_age(int age)
    {
        age_ = age;
    }

    void print_age() const
    {
        std::cout << age_ << "\n";
        // age_ = 10; // error: const function cannot modify object state
    }

private:
    int age_ = 0;
};
```

This function can modify the object:

```cpp
void set_age(int age)
```

This function only reads the object:

```cpp
void print_age() const
```

---

## Const object can only call const methods

If an object is `const`, C++ only allows calling `const` member functions.

```cpp
#include <iostream>

class Dog {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }

    void set_age(int age)
    {
        age_ = age;
    }

private:
    int age_ = 0;
};

int main()
{
    const Dog dog;

    dog.bark();    // OK
    // dog.set_age(3); // error
}
```

The object is read-only, so only read-only functions are allowed.

---

## Pass object by const reference

Use `const T&` when a function only needs to read an object.

```cpp
#include <iostream>

class Dog {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }
};

void print_dog(const Dog& dog)
{
    dog.bark();
}

int main()
{
    Dog dog;
    print_dog(dog);
}
```

This avoids copying the object and protects it from modification.

Important:

```cpp
void print_dog(const Dog& dog)
{
    dog.bark(); // bark must be const
}
```

Because `dog` is `const Dog&`, the function can only call `const` methods.

---

## Getters should usually be const

A getter usually reads object state, so it should be `const`.

```cpp
#include <string>

class Dog {
public:
    int age() const
    {
        return age_;
    }

    const std::string& name() const
    {
        return name_;
    }

private:
    int age_ = 0;
    std::string name_ = "Rex";
};
```

This means you can use the getter even when the object is const.

```cpp
void print_name(const Dog& dog)
{
    dog.name(); // OK
}
```

Returning `const std::string&` also prevents outside code from modifying the
private member through the returned reference.

---

## Const and overloads

Sometimes a class provides two versions of the same function:

- one for non-const objects
- one for const objects

```cpp
#include <string>

class Dog {
public:
    std::string& name()
    {
        return name_;
    }

    const std::string& name() const
    {
        return name_;
    }

private:
    std::string name_ = "Rex";
};
```

Usage:

```cpp
Dog dog;
dog.name() = "Max"; // non-const object, mutable reference

const Dog const_dog;
// const_dog.name() = "Max"; // error, const reference
```

This pattern is common in containers and classes that expose controlled access.

---

## Const and virtual functions

When overriding a virtual function, `const` must match.

```cpp
#include <iostream>

class Animal {
public:
    virtual void speak() const
    {
        std::cout << "animal\n";
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
```

This is correct:

```cpp
void speak() const override
```

This is not the same function:

```cpp
void speak() override // error if base function is const
```

The `const` is part of the member function signature.

---

## Mutable

`mutable` allows a member to change even inside a `const` member function.

Use it rarely.

One common use is debug counters or cached values.

```cpp
class Dog {
public:
    int call_count() const
    {
        ++call_count_;
        return call_count_;
    }

private:
    mutable int call_count_ = 0;
};
```

Even though `call_count()` is `const`, it can change `call_count_` because that
member is marked `mutable`.

This should not be used to hide normal object modification.

---

## Quick rules

| Situation | Use |
| --------- | --- |
| Method only reads object state | add `const` after the function |
| Method modifies object state | do not add trailing `const` |
| Function receives object only for reading | pass `const T&` |
| Getter reads state | make getter `const` |
| Return internal object by reference | prefer `const T&` from const getter |
| Override virtual const function | derived function must also be `const` |
| Need cache/debug counter in const function | consider `mutable`, carefully |

---

## Simple rule

If a member function only reads the object, write it like this:

```cpp
void print() const;
```

If it changes the object, write it like this:

```cpp
void set_value(int value);
```

