---
title: C++ Constructors and Object Lifetime
tags:
    - cpp
    - oop
    - constructor
    - destructor
    - lifetime
    - stack
    - heap
---

# C++ Constructors and Object Lifetime

This is the first part of C++ OOP basics.

**Series navigation:** Previous: [Constructor series](index) | Next: [Constructor overload and delegating constructors](constructor_overload_delegation)

The goal is simple:

- create an object
- see when the constructor runs
- see when the destructor runs
- compare stack and heap lifetime

## Class

A class groups data and functions together.

```cpp
class Robot {
public:
    Robot(const std::string& name);
    ~Robot();
    void say_hello() const;

private:
    std::string name_;
};
```

## Constructor

A constructor runs when the object is created.

```cpp
Robot stack_robot("stack robot");
```

In this example, the constructor receives the name and stores it in `name_`.

## Destructor

A destructor runs when the object lifetime ends.

```cpp
~Robot()
{
    std::cout << "destructor: " << name_ << "\n";
}
```

Use the destructor to release resources owned by the object.

## Stack object

Stack objects are automatic.

```cpp
{
    Robot stack_robot("stack robot");
    stack_robot.say_hello();
}
```

The constructor runs when execution reaches the object definition. The destructor
runs automatically when the scope ends.

## Heap object

Heap objects created with `new` live until you call `delete`.

```cpp
Robot* heap_robot = new Robot("heap robot");
heap_robot->say_hello();
delete heap_robot;
```

The constructor runs on `new`. The destructor runs on `delete`.

Modern C++ usually prefers smart pointers instead of raw `new` and `delete`.
This first part uses raw pointers only to show the object lifetime clearly.

## Full example

```cpp
#include <iostream>
#include <string>

class Robot {
public:
    Robot(const std::string& name)
        : name_(name)
    {
        std::cout << "constructor: " << name_ << "\n";
    }

    ~Robot()
    {
        std::cout << "destructor: " << name_ << "\n";
    }

    void say_hello() const
    {
        std::cout << "hello from " << name_ << "\n";
    }

private:
    std::string name_;
};

int main()
{
    std::cout << "start main\n";

    {
        Robot stack_robot("stack robot");
        stack_robot.say_hello();
    }

    Robot* heap_robot = new Robot("heap robot");
    heap_robot->say_hello();
    delete heap_robot;

    std::cout << "end main\n";
    return 0;
}
```

Expected output:

```text
start main
constructor: stack robot
hello from stack robot
destructor: stack robot
constructor: heap robot
hello from heap robot
destructor: heap robot
end main
```

## Run online

Copy the full example above and run it in the embedded OneCompiler C++ runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>



## Run locally

Save the example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o lifetime_demo
./lifetime_demo
```

**Series navigation:** Previous: [Constructor series](index) | Next: [Constructor overload and delegating constructors](constructor_overload_delegation)
