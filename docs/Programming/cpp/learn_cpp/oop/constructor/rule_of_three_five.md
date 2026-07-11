---
title: C++ Rule of Three and Rule of Five
tags:
    - cpp
    - oop
    - rule-of-three
    - rule-of-five
    - rule-of-zero
    - constructor
    - assignment
    - destructor
---

# C++ Rule of Three and Rule of Five

This part connects the previous posts:

**Series navigation:** Previous: [Copy assignment and move assignment](copy_move_assignment) | Next: [Destructor and cleanup](destructor)

- destructor
- copy constructor
- copy assignment
- move constructor
- move assignment

The idea is simple:

If your class owns a resource directly, you must control how it is copied,
moved, and destroyed.

Examples of resources:

- raw heap memory from `new`
- file descriptor
- socket
- mutex handle
- C library handle

## Rule of Three

If you write one of these, you often need all three:

```cpp
~ClassName();
ClassName(const ClassName& other);
ClassName& operator=(const ClassName& other);
```

These are:

| function | job |
|---|---|
| destructor | release the resource |
| copy constructor | create a new object by copying another object |
| copy assignment | copy into an object that already exists |

## Why this matters

This class is dangerous:

```cpp
class Buffer {
public:
    Buffer(std::size_t size)
        : size_(size), data_(new int[size])
    {
    }

    ~Buffer()
    {
        delete[] data_;
    }

private:
    std::size_t size_;
    int* data_;
};
```

The destructor is custom, but copy is not custom.

This is a problem:

```cpp
Buffer a(10);
Buffer b = a;
```

The compiler-generated copy constructor copies the pointer value. Now `a` and
`b` point to the same memory.

When both destructors run, both call:

```cpp
delete[] data_;
```

That causes double delete.

## Correct Rule of Three example

```cpp
#include <algorithm>
#include <cstddef>
#include <iostream>

class Buffer {
public:
    Buffer(std::size_t size)
        : size_(size), data_(new int[size])
    {
        std::cout << "constructor\n";
    }

    ~Buffer()
    {
        std::cout << "destructor\n";
        delete[] data_;
    }

    Buffer(const Buffer& other)
        : size_(other.size_), data_(new int[other.size_])
    {
        std::cout << "copy constructor\n";
        std::copy(other.data_, other.data_ + size_, data_);
    }

    Buffer& operator=(const Buffer& other)
    {
        std::cout << "copy assignment\n";

        if (this != &other) {
            int* new_data = new int[other.size_];
            std::copy(other.data_, other.data_ + other.size_, new_data);

            delete[] data_;
            data_ = new_data;
            size_ = other.size_;
        }

        return *this;
    }

    void set(std::size_t index, int value)
    {
        data_[index] = value;
    }

    int get(std::size_t index) const
    {
        return data_[index];
    }

private:
    std::size_t size_;
    int* data_;
};

int main()
{
    Buffer a(3);
    a.set(0, 42);

    Buffer b = a;

    Buffer c(2);
    c = a;

    std::cout << b.get(0) << "\n";
    std::cout << c.get(0) << "\n";

    return 0;
}
```

## Rule of Five

C++11 added move constructor and move assignment.

If your class owns a resource directly, and you need custom copy/destructor
logic, you probably also need move logic.

The five functions are:

```cpp
~ClassName();
ClassName(const ClassName& other);
ClassName& operator=(const ClassName& other);
ClassName(ClassName&& other) noexcept;
ClassName& operator=(ClassName&& other) noexcept;
```

## Rule of Five example

```cpp
class Buffer {
public:
    Buffer(std::size_t size)
        : size_(size), data_(new int[size])
    {
    }

    ~Buffer()
    {
        delete[] data_;
    }

    Buffer(const Buffer& other)
        : size_(other.size_), data_(new int[other.size_])
    {
        std::copy(other.data_, other.data_ + size_, data_);
    }

    Buffer& operator=(const Buffer& other)
    {
        if (this != &other) {
            int* new_data = new int[other.size_];
            std::copy(other.data_, other.data_ + other.size_, new_data);

            delete[] data_;
            data_ = new_data;
            size_ = other.size_;
        }

        return *this;
    }

    Buffer(Buffer&& other) noexcept
        : size_(other.size_), data_(other.data_)
    {
        other.size_ = 0;
        other.data_ = nullptr;
    }

    Buffer& operator=(Buffer&& other) noexcept
    {
        if (this != &other) {
            delete[] data_;

            size_ = other.size_;
            data_ = other.data_;

            other.size_ = 0;
            other.data_ = nullptr;
        }

        return *this;
    }

private:
    std::size_t size_;
    int* data_;
};
```

The move constructor and move assignment do not allocate new memory. They take
the pointer from `other`, then reset `other`.

This makes the moved-from object safe to destroy.

## Rule of Zero

Modern C++ prefers the Rule of Zero.

Do not own raw resources directly. Use standard library types that already know
how to copy, move, and destroy themselves.

```cpp
#include <vector>

class Buffer {
public:
    explicit Buffer(std::size_t size)
        : data_(size)
    {
    }

private:
    std::vector<int> data_;
};
```

This class does not need a custom destructor, copy constructor, copy assignment,
move constructor, or move assignment.

`std::vector` handles the resource.

## default and delete

Use `= default` when the compiler-generated function is exactly what you want:

```cpp
class Robot {
public:
    Robot() = default;
    Robot(const Robot&) = default;
    Robot& operator=(const Robot&) = default;
};
```

Use `= delete` when copying or moving should not be allowed:

```cpp
class UniqueConnection {
public:
    UniqueConnection(const UniqueConnection&) = delete;
    UniqueConnection& operator=(const UniqueConnection&) = delete;
};
```

## Rule of thumb

- If the class does not own a resource, use the Rule of Zero.
- If the class owns a raw resource and has a destructor, think Rule of Three.
- If the class should support efficient moves, think Rule of Five.
- Prefer `std::vector`, `std::string`, `std::unique_ptr`, and RAII wrapper classes.
- Avoid raw `new` and `delete` in normal application code.

## Run online

Copy the Rule of Three full example above and run it in the embedded
OneCompiler C++ runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>

Other online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

## Run locally

Save the Rule of Three full example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o rule_of_three_demo
./rule_of_three_demo
```

**Series navigation:** Previous: [Copy assignment and move assignment](copy_move_assignment) | Next: [Destructor and cleanup](destructor)
