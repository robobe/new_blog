---
title: C++ Constructor Overload and Delegating Constructors
tags:
    - cpp
    - oop
    - constructor
    - constructor-overload
    - delegating-constructor
    - initializer-list
---

# C++ Constructor Overload and Delegating Constructors

Constructor overload means one class has more than one constructor.

**Series navigation:** Previous: [Constructors and object lifetime](constructors_object_lifetime) | Next: [Default constructor, copy constructor, move constructor](default_copy_move_constructor)

Each constructor has a different parameter list.

```cpp
Robot();
Robot(const std::string& name);
Robot(const std::string& name, int battery);
```

The compiler chooses the constructor that matches the arguments.

## Initialize private members

Private members should usually be initialized in the constructor initializer
list.

```cpp
class Robot {
public:
    Robot(const std::string& name, int battery)
        : name_(name), battery_(battery)
    {
    }

private:
    std::string name_;
    int battery_;
};
```

This part initializes the private members:

```cpp
: name_(name), battery_(battery)
```

Prefer this over assigning inside the constructor body:

```cpp
Robot(const std::string& name, int battery)
{
    name_ = name;
    battery_ = battery;
}
```

The initializer list constructs the members directly.

Assignment in the body first creates the members, then changes them.

## Constructor overload

```cpp
class Robot {
public:
    Robot()
        : name_("default"), battery_(100)
    {
    }

    Robot(const std::string& name)
        : name_(name), battery_(100)
    {
    }

    Robot(const std::string& name, int battery)
        : name_(name), battery_(battery)
    {
    }

private:
    std::string name_;
    int battery_;
};
```

Usage:

```cpp
Robot a;
Robot b("alpha");
Robot c("beta", 80);
```

## Delegating constructor

A constructor can call another constructor from the same class.

This is called a delegating constructor.

```cpp
class Robot {
public:
    Robot()
        : Robot("default", 100)
    {
    }

    Robot(const std::string& name)
        : Robot(name, 100)
    {
    }

    Robot(const std::string& name, int battery)
        : name_(name), battery_(battery)
    {
    }

private:
    std::string name_;
    int battery_;
};
```

Here:

```cpp
Robot()
    : Robot("default", 100)
```

The default constructor calls the main constructor.

And:

```cpp
Robot(const std::string& name)
    : Robot(name, 100)
```

The name-only constructor also calls the main constructor.

This keeps initialization logic in one place.

## Full example

```cpp
#include <iostream>
#include <string>

class Robot {
public:
    Robot()
        : Robot("default", 100)
    {
        std::cout << "default constructor\n";
    }

    Robot(const std::string& name)
        : Robot(name, 100)
    {
        std::cout << "name constructor\n";
    }

    Robot(const std::string& name, int battery)
        : name_(name), battery_(battery)
    {
        std::cout << "main constructor\n";
    }

    void print() const
    {
        std::cout << "name: " << name_
                  << ", battery: " << battery_ << "\n";
    }

private:
    std::string name_;
    int battery_;
};

int main()
{
    Robot a;
    Robot b("alpha");
    Robot c("beta", 80);

    a.print();
    b.print();
    c.print();

    return 0;
}
```

Expected idea:

```text
main constructor
default constructor
main constructor
name constructor
main constructor
name: default, battery: 100
name: alpha, battery: 100
name: beta, battery: 80
```

The delegated constructor runs first.

Then the body of the constructor that delegated runs.

## Important rule

When a constructor delegates to another constructor, it cannot also initialize
members in the same initializer list.

This is valid:

```cpp
Robot()
    : Robot("default", 100)
{
}
```

This is not valid:

```cpp
Robot()
    : Robot("default", 100), battery_(50)
{
}
```

Choose one:

- delegate to another constructor
- initialize members directly

## Rule of thumb

- Use overloaded constructors when users need different ways to create the object.
- Use one main constructor for the real initialization.
- Make smaller constructors delegate to the main constructor.
- Initialize private members in the initializer list.
- Keep constructor body for validation, logging, or extra logic.

## Run online

Copy the full example and run it in the embedded OneCompiler C++ runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>

Other online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

## Run locally

Save the full example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o constructor_overload_demo
./constructor_overload_demo
```

**Series navigation:** Previous: [Constructors and object lifetime](constructors_object_lifetime) | Next: [Default constructor, copy constructor, move constructor](default_copy_move_constructor)
