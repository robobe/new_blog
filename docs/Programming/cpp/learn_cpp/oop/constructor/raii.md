---
title: C++ RAII Resource Acquisition Is Initialization
tags:
    - cpp
    - oop
    - raii
    - resource-management
    - destructor
    - lifetime
    - smart-pointer
---

# C++ RAII

RAII means:

**Series navigation:** Previous: [Destructor and cleanup](destructor) | Next: [Constructor series](index)

**Resource Acquisition Is Initialization**

The idea:

- acquire the resource in the constructor
- release the resource in the destructor
- let object lifetime control cleanup

RAII is one of the most important C++ ideas.

## Resource

A resource is something that must be released.

Examples:

- heap memory
- file handle
- socket
- mutex lock
- database connection
- hardware handle

## Basic idea

Without RAII:

```cpp
open_resource();
do_work();
close_resource();
```

The problem:

If `do_work()` returns early or throws an exception, `close_resource()` may not
run.

With RAII:

```cpp
{
    Resource resource;
    do_work();
}
```

When the scope ends, the destructor runs automatically.

This happens for:

- normal scope exit
- `return`
- exception

## Simple RAII class

```cpp
#include <iostream>

class ScopePrinter {
public:
    ScopePrinter()
    {
        std::cout << "acquire resource\n";
    }

    ~ScopePrinter()
    {
        std::cout << "release resource\n";
    }
};

int main()
{
    std::cout << "start\n";

    {
        ScopePrinter printer;
        std::cout << "inside scope\n";
    }

    std::cout << "end\n";
    return 0;
}
```

Output:

```text
start
acquire resource
inside scope
release resource
end
```

## File example

This class opens a file in the constructor and closes it in the destructor.

```cpp
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

class File {
public:
    File(const std::string& path, const char* mode)
        : file_(std::fopen(path.c_str(), mode))
    {
        if (file_ == nullptr) {
            throw std::runtime_error("failed to open file");
        }

        std::cout << "file opened\n";
    }

    ~File()
    {
        if (file_ != nullptr) {
            std::fclose(file_);
            std::cout << "file closed\n";
        }
    }

    void write_line(const std::string& text)
    {
        std::fprintf(file_, "%s\n", text.c_str());
    }

    File(const File&) = delete;
    File& operator=(const File&) = delete;

private:
    std::FILE* file_;
};

int main()
{
    try {
        File file("/tmp/raii_demo.txt", "w");
        file.write_line("hello from RAII");
        std::cout << "work done\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }

    return 0;
}
```

`File` cannot be copied because two objects closing the same `FILE*` would be
dangerous.

That is why copy is disabled:

```cpp
File(const File&) = delete;
File& operator=(const File&) = delete;
```

## Smart pointers are RAII

`std::unique_ptr` is RAII for heap memory.

```cpp
#include <memory>

auto robot = std::make_unique<Robot>();
```

When `robot` goes out of scope, it automatically deletes the heap object.

No manual `delete` is needed.

## Locks are RAII

`std::lock_guard` is RAII for mutex locks.

```cpp
#include <mutex>

std::mutex mutex;

void update()
{
    std::lock_guard<std::mutex> lock(mutex);
    // mutex is locked here
}
// mutex is unlocked here
```

The constructor locks the mutex.

The destructor unlocks the mutex.

## RAII and exceptions

RAII is useful because destructors run during stack unwinding.

```cpp
void run()
{
    File file("/tmp/demo.txt", "w");
    throw std::runtime_error("error");
}
```

Even when an exception is thrown, `file` is destroyed before leaving `run()`.
The destructor closes the file.

## Rule of thumb

- Put resource acquisition in the constructor.
- Put resource release in the destructor.
- Do not manually call cleanup everywhere.
- Prefer standard RAII types like `std::string`, `std::vector`, `std::unique_ptr`, and `std::lock_guard`.
- If your class owns a raw resource, think about copy, move, and destructor behavior.

## Run online

Copy the simple RAII class example and run it in the embedded OneCompiler C++
runtime.

<iframe
    src="https://onecompiler.com/embed/cpp"
    width="100%"
    height="720"
    frameborder="0">
</iframe>

Other online compilers: <a href="https://onecompiler.com/cpp" target="_blank" rel="noopener noreferrer">OneCompiler</a> | <a href="https://godbolt.org/" target="_blank" rel="noopener noreferrer">Compiler Explorer</a> | <a href="https://wandbox.org/" target="_blank" rel="noopener noreferrer">Wandbox</a>

## Run locally

Save the file example as `main.cpp`:

```bash
g++ -std=c++17 main.cpp -o raii_demo
./raii_demo
cat /tmp/raii_demo.txt
```

**Series navigation:** Previous: [Destructor and cleanup](destructor) | Next: [Constructor series](index)
