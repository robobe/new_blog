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

---

!!! info ownership
    **The course is not really about smart pointers. It is about C++ ownership and lifetime management, with smart pointers as the main tools.**

| Stage | Topic                          | What you should understand                                    |
| ----- | ------------------------------ | ------------------------------------------------------------- |
| 1     | Object lifetime and scope      | Stack objects, dynamic objects, constructors/destructors      |
| 2     | Raw pointers and references    | Pointer syntax, `nullptr`, observer vs owner                  |
| 3     | RAII                           | Why modern C++ avoids manual cleanup                          |
| 4     | `std::unique_ptr` basics       | Exclusive ownership, `make_unique`, `->`, `*`, `.get()`       |
| 5     | Moving `unique_ptr`            | `std::move`, moved-from state, ownership transfer             |
| 6     | `unique_ptr` in APIs           | Passing by reference, raw pointer, value, returning ownership |
| 7     | `unique_ptr` with containers   | `vector<unique_ptr<T>>`, polymorphic collections              |
| 8     | Advanced `unique_ptr`          | Arrays, custom deleters, `reset`, `release`, `swap`           |
| 9     | `std::shared_ptr`              | Reference counting and shared lifetime                        |
| 10    | `std::weak_ptr`                | Observation without ownership                                 |
| 11    | Cyclic ownership               | Why `shared_ptr` can still leak                               |
| 12    | Smart pointers in class design | Composition, parent/child relationships                       |
| 13    | Polymorphism                   | `unique_ptr<Base>`, virtual destructors, factories            |
| 14    | C APIs and external resources  | Files, sockets, handles, custom deleters                      |
| 15    | PImpl and incomplete types     | Real library/API design                                       |
| 16    | Ownership-oriented API design  | Deciding between `T&`, `T*`, `unique_ptr`, `shared_ptr`       |
| 17    | Anti-patterns and debugging    | Double delete, dangling pointers, unnecessary `shared_ptr`    |
| 18    | Final project                  | Design a small ownership-heavy application                    |


!!! info "know the different"
    ```cpp
    void process(Camera& camera);
    void process(Camera* camera);
    void process(std::unique_ptr<Camera> camera);
    void process(std::unique_ptr<Camera>& camera);
    void process(std::shared_ptr<Camera> camera);
    ```
    

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

<div class="grid-container">
    <div class="grid-item">
        <a href="01_object_lifetime_and_scope">
            <p>1. Object Lifetime and Scope</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="02_raw_pointers_and_references">
            <p>2. Raw Pointers and References</p>
        </a>
    </div>
</div>
