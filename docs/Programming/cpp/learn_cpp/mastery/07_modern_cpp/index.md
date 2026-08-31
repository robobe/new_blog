---
title: Modern C++
tags:
    - cpp
    - modern-cpp
    - cpp11
    - cpp17
    - cpp20
    - cpp23
---

# Modern C++

Modern C++ uses language and library features introduced since C++11 to express ownership, types, errors, and algorithms more clearly and safely.

## Available lessons

<div class="grid-container">
    <div class="grid-item">
        <a href="auto/">
            <p><code>auto</code> Type Deduction</p>
        </a>
        <details>
            <summary>More...</summary>
            <p>Deduce types while controlling copies, references, and constness.</p>
        </details>
    </div>
    <div class="grid-item">
        <a href="structure_binding/">
            <p>Structured Bindings</p>
        </a>
        <details>
            <summary>More...</summary>
            <p>Decompose structures, arrays, pairs, and tuples into named values.</p>
        </details>
    </div>
    <div class="grid-item">
        <a href="optional/">
            <p><code>std::optional</code></p>
        </a>
        <details>
            <summary>More...</summary>
            <p>Represent values that may be absent without sentinel values.</p>
        </details>
    </div>
    <div class="grid-item">
        <a href="span/">
            <p><code>std::span</code></p>
        </a>
        <details>
            <summary>More...</summary>
            <p>Use safe, non-owning views over contiguous sequences.</p>
        </details>
    </div>
    <div class="grid-item">
        <a href="constexpr/">
            <p><code>constexpr</code></p>
        </a>
        <details>
            <summary>More...</summary>
            <p>Calculate and validate suitable values during compilation.</p>
        </details>
    </div>
</div>

## Topics to cover next

| Priority | Topic | Standard | What it adds |
| --- | --- | --- | --- |
| 1 | Move semantics and perfect forwarding | C++11 | Transfer resources efficiently and preserve value categories. |
| 2 | `std::variant` and `std::visit` | C++17 | Model a value that can safely be one of several known types. |
| 3 | Concepts and constraints | C++20 | State template requirements and produce clearer diagnostics. |
| 4 | Ranges and views | C++20 | Compose lazy transformations and reduce iterator boilerplate. |
| 5 | Coroutines | C++20 | Express resumable operations used by generators and asynchronous code. |
| 6 | `std::jthread` and stop tokens | C++20 | Manage thread lifetime and cooperative cancellation safely. |
| 7 | `std::format` | C++20 | Build type-safe formatted text without stream chains. |
| 8 | `std::expected` | C++23 | Return either a value or structured error information. |

### Later topics

- Smart pointers and RAII ownership patterns
- Variadic templates and fold expressions
- Template argument deduction and deduction guides
- Three-way comparison with `<=>`
- Modules and current toolchain support
- `std::filesystem`, `std::chrono`, and calendar utilities
- `std::any`, type erasure, and when not to use them
- `std::mdspan` for multidimensional non-owning views

<!-- post-content-skill: 1.0.0 -->
