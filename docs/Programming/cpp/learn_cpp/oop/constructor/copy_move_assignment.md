---
title: C++ Copy Assignment and Move Assignment
tags:
    - cpp
    - oop
    - assignment
    - copy-assignment
    - move-assignment
    - lifetime
---

# C++ Copy Assignment and Move Assignment

This is the next part after default, copy, and move constructors.

**Series navigation:** Previous: [Default constructor, copy constructor, move constructor](default_copy_move_constructor) | Next: [The Rule of Three / Rule of Five](rule_of_three_five)

The important difference:

- Constructor: creates a new object.
- Assignment operator: changes an object that already exists.

## Constructor vs assignment

```cpp
Robot a("alpha");

Robot b = a;      // copy constructor: b is created here

Robot c("charlie");
c = a;            // copy assignment: c already exists
```

Move has the same idea:

```cpp
Robot d = Robot("delta"); // move constructor, or copy elision

Robot e("echo");
e = Robot("temp");        // move assignment: e already exists
```

## Copy assignment

Copy assignment copies the value from one existing object into another existing
object.

Typical signature:

```cpp
Robot& operator=(const Robot& other);
```

Why return `Robot&`?

So chained assignment can work:

```cpp
a = b = c;
```

Why `const Robot&`?

- `const` means we should not modify the source object.
- `&` avoids copying the object before assignment.

## Move assignment

Move assignment takes resources from another existing object.

Typical signature:

```cpp
Robot& operator=(Robot&& other) noexcept;
```

`Robot&&` can bind to temporary objects and to objects passed through
`std::move`.

```cpp
Robot a("alpha");
Robot b("beta");

a = std::move(b);
```

After this, `b` is still alive and valid, but its value may be empty or changed.

## Self assignment

This can happen:

```cpp
a = a;
```

For simple classes that use `std::string`, `std::vector`, and smart pointers,
the compiler-generated assignment operators usually handle this.

For classes that manually own raw memory, you must be careful not to delete the
resource before copying from it.

## Full example

```cpp
#include <iostream>
#include <string>
#include <utility>

class Robot {
public:
    Robot()
        : name_("default")
    {
        std::cout << "default constructor: " << name_ << "\n";
    }

    Robot(const std::string& name)
        : name_(name)
    {
        std::cout << "name constructor: " << name_ << "\n";
    }

    Robot(const Robot& other)
        : name_(other.name_)
    {
        std::cout << "copy constructor: " << name_ << "\n";
    }

    Robot(Robot&& other) noexcept
        : name_(std::move(other.name_))
    {
        std::cout << "move constructor: " << name_ << "\n";
    }

    Robot& operator=(const Robot& other)
    {
        std::cout << "copy assignment: " << other.name_ << "\n";

        if (this != &other) {
            name_ = other.name_;
        }

        return *this;
    }

    Robot& operator=(Robot&& other) noexcept
    {
        std::cout << "move assignment: " << other.name_ << "\n";

        if (this != &other) {
            name_ = std::move(other.name_);
        }

        return *this;
    }

    ~Robot()
    {
        std::cout << "destructor: " << name_ << "\n";
    }

    void print() const
    {
        std::cout << "robot name: " << name_ << "\n";
    }

private:
    std::string name_;
};

int main()
{
    std::cout << "start\n";

    Robot a("alpha");
    Robot b("beta");

    Robot c = a;

    b = a;

    Robot d("delta");
    d = std::move(a);

    std::cout << "after move, a is still valid:\n";
    a.print();

    std::cout << "end\n";
    return 0;
}
```

## Expected idea

This line creates a new object, so it uses the copy constructor:

```cpp
Robot c = a;
```

This line changes an existing object, so it uses copy assignment:

```cpp
b = a;
```

This line changes an existing object using move assignment:

```cpp
d = std::move(a);
```

`std::move(a)` does not destroy `a`. It only allows the move assignment operator
to take resources from `a`.

## Rule of thumb

- Use copy constructor when creating a new object from another object.
- Use copy assignment when both objects already exist.
- Use move constructor when creating a new object from a temporary or movable object.
- Use move assignment when the target object already exists and can take resources.
- Prefer compiler-generated copy and move operations when your class only uses standard library members.

## Run online

Copy the full example above and run it in the embedded OneCompiler C++ runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>

Other online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

## Run locally

Save the example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o assignment_demo
./assignment_demo
```

**Series navigation:** Previous: [Default constructor, copy constructor, move constructor](default_copy_move_constructor) | Next: [The Rule of Three / Rule of Five](rule_of_three_five)
