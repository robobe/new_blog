---
title: C++ Shallow Copy and Deep Copy
tags:
    - cpp
    - oop
    - copy-constructor
    - memory
---

# C++ shallow copy and deep copy

Copying an object means creating a new object from an existing object.

For simple classes, the compiler-generated copy is usually fine.

For classes that own dynamic memory, the compiler-generated copy can be
dangerous because it performs a shallow copy.

---

## Shallow copy

A shallow copy copies the member values exactly.

If a member is a pointer, only the pointer address is copied.
The data behind the pointer is not copied.

That means two objects can point to the same memory.

```cpp
class Buffer {
public:
    int* data;
};
```

If `data` points to heap memory, a shallow copy creates this situation:

```text
buffer1.data ---> [10]
buffer2.data ----^
```

Both objects point to the same `int`.

---

## Problem: no copy constructor implementation

In this example, the class owns memory allocated with `new`.

But it does not implement a copy constructor.

```cpp
#include <iostream>

class Buffer {
public:
    Buffer(int value)
    {
        data_ = new int(value);
    }

    ~Buffer()
    {
        delete data_;
    }

    void set(int value)
    {
        *data_ = value;
    }

    void print() const
    {
        std::cout << *data_ << "\n";
    }

private:
    int* data_ = nullptr;
};

int main()
{
    Buffer a(10);

    Buffer b = a; // compiler-generated copy constructor

    b.set(20);

    a.print(); // prints 20, not 10
    b.print(); // prints 20
}
```

The compiler-generated copy constructor copies `data_` as an address.

So both objects contain the same pointer value.

```text
a.data_ ---> [20]
b.data_ ----^
```

This creates two problems:

- changing `b` also changes `a`
- when both objects are destroyed, both destructors call `delete` on the same pointer

The second problem is a double delete, which is undefined behavior.
The program may crash, appear to work, or fail later.

---

## Deep copy

A deep copy creates new owned memory for the new object.

The pointer address is different, but the value is copied.

```text
a.data_ ---> [10]
b.data_ ---> [10]
```

Now each object owns its own memory.

---

## Fix: implement a copy constructor

```cpp
#include <iostream>

class Buffer {
public:
    Buffer(int value)
    {
        data_ = new int(value);
    }

    Buffer(const Buffer& other)
    {
        data_ = new int(*other.data_);
    }

    ~Buffer()
    {
        delete data_;
    }

    void set(int value)
    {
        *data_ = value;
    }

    void print() const
    {
        std::cout << *data_ << "\n";
    }

private:
    int* data_ = nullptr;
};

int main()
{
    Buffer a(10);

    Buffer b = a; // calls Buffer(const Buffer& other)

    b.set(20);

    a.print(); // prints 10
    b.print(); // prints 20
}
```

The important line is:

```cpp
data_ = new int(*other.data_);
```

This allocates a new `int` and copies the value from `other`.

Now `a` and `b` are independent objects.

---

## Copy constructor signature

The copy constructor usually looks like this:

```cpp
ClassName(const ClassName& other)
```

Example:

```cpp
Buffer(const Buffer& other)
```

Use `const ClassName&` because:

- `const` means the source object will not be modified
- `&` avoids copying the source object again

---

## Important: copy assignment is a separate function

This calls the copy constructor:

```cpp
Buffer b = a;
```

This calls the copy assignment operator:

```cpp
Buffer b(0);
b = a;
```

If a class owns raw memory and implements a destructor and copy constructor,
it usually also needs a copy assignment operator.

This is called the Rule of Three.

---

## Modern C++ note

Prefer standard library ownership types instead of raw `new` and `delete`.

For one value, store the value directly:

```cpp
class Buffer {
private:
    int data_ = 0;
};
```

For dynamic arrays, prefer `std::vector`.

For exclusive heap ownership, prefer `std::unique_ptr`.

Manual deep copy is still important to understand because it explains why
copy constructors, destructors, ownership, and the Rule of Three exist.
