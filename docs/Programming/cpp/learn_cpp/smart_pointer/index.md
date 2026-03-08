---
title: Smart Pointers
tags:
    - cpp
    - smart pointer
    - memory
---

Smart pointers are objects that behave like **raw pointers** but automatically manage the **lifetime** of dynamically allocated memory.

`<memory>` library has three type of smart pointer:
- std::unique_ptr
- std::shared_ptr
- std::weak_ptr

### std::unique_ptr
Only one owner, memory is deleted automatically when pointer goes out of scope

```cpp
#include <iostream>
#include <memory>

class MyClass {
private:
  int value_;

public:
  MyClass(){
    std::cout << "CREATE MY CLASS." << std::endl;
  }

  MyClass(int value) : value_(value) {
    std::cout << "CREATE MY CLASS WITH VALUE: " << value_ << std::endl;
  }
  ~MyClass(){
    std::cout << "DESTROY MY CLASS." << std::endl;
  }


};

int main() {
  auto obj = std::make_unique<MyClass>(5);

  return 0;
}
```

---

### std::shared_ptr
Multiple owners, Internally uses a **reference counter** memory is deleted when counter reaches 0.


---

### std::weak_ptr
Non-owning observer of a `shared_ptr`, does't increase reference count

## TODO: explain usage and more

---

## RAII

Resource Acquisition Is Initialization

!!! info ""
    A resource is acquired in the constructor and released in the destructor

### Resource
A resource is anything that must be release manually

| Resource    | Acquire    | Release    |
| ----------- | ---------- | ---------- |
| heap memory | `new`      | `delete`   |
| file        | `open()`   | `close()`  |
| mutex       | `lock()`   | `unlock()` |
| socket      | `socket()` | `close()`  |
| GPU memory  | allocate   | free       |


### Demo:

```cpp title="without RAII (memory leak)
void example()
{
    int* p = new int(5);

    throw std::runtime_error("error");

    delete p;
}
```

```cpp title="with RAII"
void example()
{
    std::unique_ptr<int> p = std::make_unique<int>(5);

    throw std::runtime_error("error");
}
```

Memory released when unique_ptr out of scope


---

## Reference
- [Back to Basics: C++ Smart Pointers - David Olsen - CppCon 2022](https://youtu.be/YokY6HzLkXs)