---
title: Cpp arrays
tags:
    - cpp
    - array
    - object
    - pointer
---

# Arrays in C++

- An array stores many values of the same type.
- Each value has an index.
- The first index is always `0`.

```text
index:  0   1   2
value: 10  20  30
```

---

## Array of objects

```cpp
#include <iostream>

class A {
public:
    A() { }

};

int main()
{
    A arr1[3];
    A* arr2 = new A[4];
    A** arr3 = new A*[5];
}
```

The code creates three different kinds of arrays.

| Code | What is allocated? | Where is it allocated? | Is `A()` called? |
| ---- | ------------------ | ---------------------- | ---------------- |
| `A arr1[3];` | 3 real `A` objects | automatic storage, usually stack | Yes, 3 times |
| `A* arr2 = new A[4];` | 4 real `A` objects | heap | Yes, 4 times |
| `A** arr3 = new A*[5];` | 5 pointers to `A` | heap | No `A` objects are created |

`arr1` is an array of objects.

```cpp
A arr1[3];
```

This creates 3 `A` objects immediately. The default constructor `A()` is called once for each object.

`arr2` is a pointer to the first object in a dynamic array.

```cpp
A* arr2 = new A[4];
```

This creates 4 `A` objects on the heap. The default constructor `A()` is also called once for each object.

Because `arr2` uses `new[]`, you must release it with `delete[]`.

```cpp
delete[] arr2;
```

`arr3` is different.

```cpp
A** arr3 = new A*[5];
```

This creates an array of 5 pointers. It does not create any `A` objects. It only creates places that can store addresses of `A` objects.

Memory layout after this line:

```cpp
A** arr3 = new A*[5];
```

```text
stack                         heap
-----                         ----

arr3
 |
 | points to
 v
        +---------+
        | arr3[0] |  pointer slot, no A object yet
        +---------+
        | arr3[1] |  pointer slot, no A object yet
        +---------+
        | arr3[2] |  pointer slot, no A object yet
        +---------+
        | arr3[3] |  pointer slot, no A object yet
        +---------+
        | arr3[4] |  pointer slot, no A object yet
        +---------+
```

At this stage, only the pointer array exists.

No `A` constructor was called for `arr3`.

To create real objects for `arr3`, each pointer must point to an object.

```cpp
arr3[0] = new A();
arr3[1] = new A();
```

Memory layout after this code:

```cpp
arr3[0] = new A();
arr3[1] = new A();
```

```text
stack                         heap
-----                         ----

arr3
 |
 | points to pointer array
 v
        +---------+        +----------+
        | arr3[0] | -----> | A object |
        +---------+        +----------+
        | arr3[1] | -----> | A object |
        +---------+        +----------+
        | arr3[2] |        no object
        +---------+
        | arr3[3] |        no object
        +---------+
        | arr3[4] |        no object
        +---------+
```

### Use object from pointer array

`arr3[0]` is a pointer to `A`.

Use `->` to call a method through the pointer.

```cpp
arr3[0]->hello();
```

This is the same as:

```cpp
(*arr3[0]).hello();
```

Read it like this:

```cpp
arr3[0]     // first pointer in the array
*arr3[0]    // the A object behind that pointer
```

### Clean
```cpp
delete arr3[0];
delete arr3[1];
delete[] arr3;
```

!!! warning
    If you use `new`, you must later use `delete`. If you use `new[]`, you must later use `delete[]`.

---


## Pointer array with nullptr

A pointer can point to nothing.

That value is `nullptr`.

```cpp
#include <iostream>

class A {
public:
    void hello() const
    {
        std::cout << "hello from A\n";
    }
};

int main()
{
    A** arr3 = new A*[5];

    // Set every pointer to nullptr first.
    for (int i = 0; i < 5; ++i) {
        arr3[i] = nullptr;
    }

    // Create objects only in some indexes.
    arr3[0] = new A();
    arr3[2] = new A();

    // Check every pointer before using it.
    for (int i = 0; i < 5; ++i) {
        if (arr3[i] != nullptr) {
            arr3[i]->hello();
        } else {
            std::cout << "arr3[" << i << "] is empty\n";
        }
    }

    // Delete only the objects that exist.
    for (int i = 0; i < 5; ++i) {
        if (arr3[i] != nullptr) {
            delete arr3[i];
            arr3[i] = nullptr;
        }
    }

    delete[] arr3;
}
```

`arr3` is the array of pointers.

`arr3[i]` is one pointer inside the array.

Before calling a method with `->`, check that the pointer is not `nullptr`.

```cpp
if (arr3[i] != nullptr) {
    arr3[i]->hello();
}
```

You can also write the same check shorter:

```cpp
if (arr3[i]) {
    arr3[i]->hello();
}
```

Both mean:

> use the object only if the pointer points to a real object.

---

## Modern C++: std::array

`std::array` is usually better than a C-style array when the size is fixed.

`std::array` helps because it manages the array lifetime automatically.

You do not call `new`.

You do not call `delete`.

When the `std::array` variable goes out of scope, all objects inside it are destroyed automatically.

```cpp
void example()
{
    std::array<A, 3> arr;

    // use arr here
} // arr goes out of scope here, A destructor is called for each object
```

This is safer than:

```cpp
A* arr = new A[3];

// use arr here

delete[] arr;
```

With `new[]`, you must remember to write `delete[]`. With `std::array`, C++ does the cleanup for you.

Important: `std::array` has a fixed size. If you need the array size to change while the program runs, use `std::vector`.

```cpp
#include <array>
#include <iostream>

int main()
{
    std::array<int, 3> numbers = {10, 20, 30};

    std::cout << numbers[0] << "\n";    // 10
    std::cout << numbers.size() << "\n"; // 3
}
```

`std::array<int, 3>` means:

> an array of 3 integers.

---

## Modern C++: std::vector

Use `std::vector` when the size can grow.

```cpp
#include <iostream>
#include <vector>

int main()
{
    std::vector<int> numbers = {10, 20, 30};

    numbers.push_back(40);

    std::cout << numbers[0] << "\n"; // 10
    std::cout << numbers[3] << "\n"; // 40
}
```

`push_back` adds a new value at the end.

---

## Quick rules

| Need | Use |
| ---- | --- |
| Fixed number of simple values | `std::array<int, 3>` |
| Number of values can grow | `std::vector<int>` |
| Fixed number of objects | `std::array<Robot, 2>` |
| Objects may be optional | array/vector of pointers |
| Beginner example only | C-style array is OK |

For real projects, prefer `std::array` and `std::vector` over C-style arrays.
