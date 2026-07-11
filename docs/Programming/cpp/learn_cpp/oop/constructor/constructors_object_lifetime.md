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

OneCompiler does not expose a documented iframe parameter for preloading source
code. If you need the code preloaded, use the Compiler Explorer link below.

Other online compilers: <a href="https://godbolt.org/clientstate/eJyVU9tOwzAM%2FRXLSGhjGxq8gLLBLyDxSqcpS00X0SZVLlxUjW8nF6DdkBgkL83J8bF9GndoyVqplUX20KEskV1Mseaq8rwiZCgmE5yi1d6IeDyRStS%2BJFhKbZ0h3twWqgcDJFUVoCIg3Fq41xvtoCtU6ze1FKxQEFZCRyJkdWBdyViOOwXFGxpnTlwsAevRAO762xQptHewXEKBSc544bRh4RjBFJ1vi1ATLnLwLtYXP95zIUelS%2Fqv8rOWJVj%2Btt5SXevRGFJ1x%2FIkMjwa3fwhTWvkM3f0aenAxhwVqLtF5EnloOFSxTa7AblPax03mfOV5Eeh%2BT8Gonham2RaChNPkE4FjhfDrr5p5wMPDj1KmmewJd5mMtyAopfP1xHd4O2hfE%2Be3f6ULqkmRwPSdysHHZMq9%2FqNFEPOGwXzaFyhwqMXumllTaYfDayuL8OFbl2eGJwF3ZswIhdXMLub4241RXol4cNLyVHcVL4h5SJ5IIlsT7CWm0hf%2FSYdhtCVUkWd3SrsDzgPLQ8%3D" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

## Run locally

Save the example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o lifetime_demo
./lifetime_demo
```

**Series navigation:** Previous: [Constructor series](index) | Next: [Constructor overload and delegating constructors](constructor_overload_delegation)
