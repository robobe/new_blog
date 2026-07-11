---
title: C++ Default Copy and Move Constructors
tags:
    - cpp
    - oop
    - constructor
    - copy-constructor
    - move-constructor
    - lifetime
---

# C++ Default, Copy and Move Constructors

This is the next part after object lifetime.

**Series navigation:** Previous: [Constructor overload and delegating constructors](constructor_overload_delegation) | Next: [Copy assignment and move assignment](copy_move_assignment)

The goal is to understand which constructor runs when an object is created,
copied, or moved.

## Constructor types

| constructor | when it runs |
|---|---|
| [Default constructor](#default-constructor) | Create an object without arguments. |
| [Copy constructor](#copy-constructor) | Create a new object from an existing object. |
| [Move constructor](#move-constructor) | Create a new object by taking resources from a temporary object. |

## Default constructor

A default constructor can be called without arguments.

```cpp
Robot robot;
```

Example:

```cpp
class Robot {
public:
    Robot()
    {
        std::cout << "default constructor\n";
    }
};
```

If a class has no constructors, the compiler can create a default constructor.
If you write another constructor, the compiler does not always create a default
constructor for you.

You can ask the compiler to create one:

```cpp
Robot() = default;
```

## Copy constructor

A copy constructor creates a new object from an existing object.

```cpp
Robot r1("alpha");
Robot r2 = r1;
```

The source object `r1` is still valid after the copy.

Typical signature:

!!! tip ""
    copy constructor use reference with const that we can't change the source object by mistake

```cpp
Robot(const Robot& other);
```

Use `const Robot&` because:

- `const` means the copy constructor should not change the source object.
- `&` avoids copying the object again while trying to copy it.

## Move constructor

A move constructor creates a new object by taking data from a temporary object.

```cpp
Robot r3 = Robot("temporary");
```

It can also run when using `std::move`:

```cpp
Robot r4 = std::move(r3);
```

Typical signature:

```cpp
Robot(Robot&& other) noexcept;
```

`&&` means rvalue reference. It can bind to temporary objects and to objects
converted with `std::move`.

**After moving, the moved-from object must still be valid, but its value may be
empty or changed.**

---

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

Robot make_robot()
{
    return Robot("factory robot");
}

int main()
{
    std::cout << "start\n";

    Robot a;
    Robot b("beta");

    Robot c = b;
    Robot d = std::move(b);

    Robot e = make_robot();

    std::cout << "after move, b is still valid:\n";
    b.print();

    std::cout << "end\n";
    return 0;
}
```

## Notes about output

Modern C++ compilers are allowed to skip some copies and moves using copy
elision. This means the output can be shorter than expected.

For learning, you can disable copy elision with:

```bash
g++ -std=c++17 -fno-elide-constructors main.cpp -o constructors_demo
./constructors_demo
```

**Series navigation:** Previous: [Constructor overload and delegating constructors](constructor_overload_delegation) | Next: [Copy assignment and move assignment](copy_move_assignment)

Without `-fno-elide-constructors`, this line may not print a move constructor:

```cpp
Robot e = make_robot();
```

That is normal.

## Rule of thumb

- Use the default constructor when an object can start with a normal empty/default state.
- Use the copy constructor when the new object should have the same value as another object.
- Use the move constructor when the new object can take **resources** from a temporary object.
- Prefer normal values and standard library types first. Write custom copy/move constructors only when the class owns a resource directly.

## Note about move

!!! info ""
    Move does not destroy the source object.

  After this line:

```cpp
  Robot d = std::move(b);
```
  b is still alive because it was created on the stack and its lifetime has not ended. What changed is that its internal resources may have been transferred to d.

  For example, if Robot owns a std::string name, the move constructor may move the string buffer from b.name into d.name.

  So after the move:
```cpp
  std::cout << b.name << "\n";
```
  
b.name is still safe to access, but its value is unspecified. Commonly it becomes an empty string, but you should not rely on that.

  
  
---

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
g++ -std=c++17 main.cpp -o constructors_demo
./constructors_demo
```

For learning copy and move behavior:

```bash
g++ -std=c++17 -fno-elide-constructors main.cpp -o constructors_demo
./constructors_demo
```

Online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>
