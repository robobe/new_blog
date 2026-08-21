---
title: C++ Mastery Syllabus
tags:
    - cpp
    - learning-path
    - syllabus
    - modern-cpp
---

Becoming strong at C++ means more than learning syntax. You need to understand
object lifetime, ownership, the standard library, generic programming,
concurrency, build systems, testing, debugging, and performance. This syllabus
organizes the existing blog posts into a learning order and identifies the
missing lessons.

Status used below:

- **Available**: a relevant post already exists.
- **Partial**: useful material exists but needs more coverage.
- **Missing**: create and study this subject before considering the phase
  complete.

!!! tip "How to use this syllabus"
    Complete the phases in order. Read the linked posts, type the examples
    yourself, and finish the mission without copying a final solution. Keep each
    project in Git and add tests as soon as testing is introduced.

<div class="grid-container">
    <div class="grid-item">
        <a href="mastery/">
            <p>Start the C++ Mastery Curriculum</p>
        </a>
        <details>
            <summary>More...</summary>
            <p>
                Follow numbered C++20 lessons with runnable examples, CTest,
                quizzes, exercises, and phase missions.
            </p>
        </details>
    </div>
</div>

## Current content review

The strongest existing areas are:

- function arguments, references, pointers, and `const`;
- constructors, destructors, copy/move operations, and RAII;
- inheritance, polymorphism, operator overloading, and class templates;
- arrays, common sequence containers, files, and lambdas.

The highest-priority gaps are:

1. Core syntax, types, scope, control flow, functions, enums, and structures
2. `std::string`, iterators, algorithms, and associative containers
3. Headers, translation units, linking, namespaces, and the One Definition Rule
4. Exceptions, error design, undefined behavior, and defensive programming
5. Complete smart-pointer ownership and cycle handling
6. Testing strategy, debugging, sanitizers, static analysis, and profiling
7. Modern C++ facilities: `optional`, `variant`, `span`, ranges, concepts,
   `filesystem`, and `chrono`
8. Complete concurrency: atomics, futures, `jthread`, cancellation, deadlocks,
   and the memory model
9. API design, project architecture, dependency management, and performance

---

## Phase 0: Build and debugging tools

Learn how source code becomes an executable before studying larger programs.

| Subject | Status | Material |
| --- | --- | --- |
| Compiler warnings and debug builds | Partial | [Development environment](../dev_env/index.md) |
| CMake targets and libraries | Available | [CMake](../cmake/index.md) |
| CMake presets | Available | [CMake presets](../cmake/cmake_preset/index.md) |
| Debugger: breakpoints, stack, variables | Available | [Toolchain and debugging](mastery/01_toolchain_and_debugging/index.md) |
| Compiler, assembler, linker, and loader | Available | [Toolchain and debugging](mastery/01_toolchain_and_debugging/index.md) |
| Headers, translation units, namespaces, ODR | Available | [Toolchain and debugging](mastery/01_toolchain_and_debugging/index.md) |
| Git and small reviewable commits | Partial | Use Git for every mission. |

### Mission 0: Multi-file hello application

Build an executable from `main.cpp` and a small library containing a header and
source file.

Completion criteria:

- Configure and build with CMake.
- Enable `-Wall -Wextra -Wpedantic`.
- Run from a VS Code task and from the terminal.
- Set a debugger breakpoint inside the library.
- Explain the difference between declaration, definition, compilation, and
  linking.

---

## Phase 1: Language foundations

Learn to express simple programs clearly before using advanced abstractions.

| Subject | Status | Material |
| --- | --- | --- |
| Fundamental types, initialization, conversions | Missing | Include narrowing and signed/unsigned rules. |
| Operators and expressions | Missing | Include precedence and short-circuit evaluation. |
| `if`, `switch`, loops, and scope | Missing | Include range-based `for`. |
| Functions and return values | Partial | [Function arguments](function_arguments/index.md) |
| Value, reference, and pointer semantics | Available | [Value, reference, and pointer](val_ref_pointer/index.md) |
| Arrays and bounds | Available | [Arrays](arrays/index.md) |
| [`std::string` and `std::string_view`](mastery/04_text_and_data_modeling/index.md) | Available | Learn ownership, non-owning views, and lifetime safety. |
| [`enum class`, `struct`, and simple data modeling`](mastery/04_text_and_data_modeling/index.md) | Available | Model closed choices, related data, and simple invariants. |
| `const` fundamentals | Available | [`const` and variables](oop/const_and_variables/index.md) |

### Mission 1: Command-line text analyzer

Create a program that reads a sentence and reports characters, words, lines,
and the most common word.

Completion criteria:

- Split logic into small functions.
- Use `std::string`, `std::vector`, and `const` references.
- Reject invalid input with a clear message.
- Add at least five hand-written test cases in a Markdown test log.
- Build with warnings and fix every warning.

---

## Phase 2: Lifetime, ownership, and value semantics

This phase is the center of modern C++. Know who owns every resource and when
every object is destroyed.

| Subject | Status | Material |
| --- | --- | --- |
| Object construction and lifetime | Available | [Constructors and lifecycle](oop/constructor/index.md) |
| Destructors and scope cleanup | Available | [Destructor and cleanup](oop/constructor/destructor.md) |
| RAII | Available | [RAII](oop/constructor/raii.md) |
| Copy and move construction | Available | [Default, copy, and move constructors](oop/constructor/default_copy_move_constructor.md) |
| Copy and move assignment | Available | [Copy and move assignment](oop/constructor/copy_move_assignment.md) |
| Rule of Zero, Three, and Five | Available | [Rule of Three and Five](oop/constructor/rule_of_three_five.md) |
| Shallow and deep copy | Available | [Shallow and deep copy](oop/shallow_deep_copy/index.md) |
| Smart pointers and ownership | Partial | [Smart pointers](smart_pointer/index.md) needs `weak_ptr`, cycles, factories, and custom deleters. |
| Move semantics and perfect forwarding | Partial | [Function arguments](function_arguments/index.md#pass-by-rvalue-reference-and-move) and [move-semantics talk](https://youtu.be/Bt3zcJZIalk){:target="_blank" rel="noopener noreferrer"} |

### Mission 2: RAII resource package

Implement a small library that owns a file or byte buffer and cannot leak it.

Completion criteria:

- Prefer the Rule of Zero.
- Make ownership explicit with values or `std::unique_ptr`.
- Demonstrate copy behavior or deliberately delete copying.
- Demonstrate move construction and move assignment.
- Prove cleanup occurs on normal return and during an exception.

---

## Phase 3: Standard library and algorithms

Do not recreate containers and algorithms for production code. Learn their
complexity, invalidation rules, and correct use.

| Subject | Status | Material |
| --- | --- | --- |
| Sequence containers | Partial | [Data structures](data_structure/index.md) and [arrays](arrays/index.md) |
| Files and streams | Available | [Files and streams](files/index.md) |
| Lambdas and captures | Available | [Lambda expressions](lambda_expression/index.md) |
| Iterators and iterator categories | Missing | Required before mastering algorithms. |
| Standard algorithms | Missing | Cover `find`, `transform`, `sort`, `accumulate`, predicates. |
| `map`, `set`, `unordered_map`, `unordered_set` | Missing | Include ordering, hashing, and complexity. |
| Container invalidation | Missing | Explain when pointers, references, and iterators become invalid. |
| `optional`, `variant`, and `tuple` | Partial | Tuple exists in [data structures](data_structure/index.md#stdtuple); add `optional` and `variant`. |
| `filesystem`, `chrono`, and random numbers | Missing | Teach portable paths, time, and engines. |

### Mission 3: File-backed address book

Create an address book that loads records, searches them, sorts them, and saves
them again.

Completion criteria:

- Use a structure for each record.
- Use at least one sequence container and one associative container.
- Use standard algorithms instead of index-based loops where appropriate.
- Document algorithmic complexity for lookup, insertion, and sorting.
- Handle missing and malformed files without undefined behavior.

---

## Phase 4: Classes, interfaces, and design

Use classes to maintain invariants and express ownership—not merely to hide
fields behind getters and setters.

| Subject | Status | Material |
| --- | --- | --- |
| Encapsulation and class basics | Partial | [OOP](oop/index.md) and [getter/setter](oop/getter_setter.md) |
| Const-correct interfaces | Available | [Const correctness](oop/const_correctness/index.md) |
| Inheritance | Available | [Inheritance](oop/inheritance/index.md) |
| Virtual functions | Available | [Virtual functions](oop/inheritance/virtual_function/index.md) |
| Runtime polymorphism | Available | [Polymorphism](oop/polymorphism/index.md) |
| Abstract interfaces | Available | [Abstract base classes](oop/polymorphism/abstract_base_class.md) |
| Virtual destructors | Available | [Virtual destructors](oop/polymorphism/virtual_destructor.md) |
| Safe casts | Available | [`dynamic_cast`](oop/polymorphism/dynamic_cast/index.md) |
| Operator overloading | Available | [Operator overloading](oop/operator_overload/index.md) |
| Composition, dependency injection, SOLID | Missing | Prefer composition unless substitution is genuine. |
| API invariants and strong types | Missing | Prevent invalid states instead of checking them everywhere. |

### Mission 4: Sensor processing framework

Design a small program with a sensor interface and two implementations, such as
a simulated temperature sensor and a file-backed sensor.

Completion criteria:

- Own implementations with `std::unique_ptr`.
- Use a virtual destructor and `override`.
- Inject dependencies rather than creating them inside processing classes.
- Add a fake implementation for tests.
- Explain why inheritance is appropriate—or replace it with composition.

---

## Phase 5: Generic and compile-time programming

Templates should express reusable constraints, not move every implementation
into an unreadable abstraction.

| Subject | Status | Material |
| --- | --- | --- |
| Function and class templates | Available | [Template classes and techniques](oop/templates/index.md) |
| Template deduction and specialization | Partial | Extend the existing template material. |
| `constexpr` and compile-time evaluation | Missing | Include `consteval` and `constinit` briefly. |
| Type traits and `static_assert` | Partial | `static_assert` exists; add standard traits. |
| C++20 concepts and constraints | Missing | Replace unclear substitution errors with readable constraints. |
| Variadic templates and fold expressions | Missing | Teach only after ordinary templates. |
| Ranges and views | Missing | Include lifetime risks of lazy, non-owning views. |

### Mission 5: Generic statistics library

Build a library that calculates minimum, maximum, mean, and filtered results
for several numeric container types.

Completion criteria:

- Constrain accepted element types with a C++20 concept.
- Accept iterator pairs or ranges without copying the input.
- Add compile-time tests with `static_assert`.
- Add runtime tests for empty and non-empty data.
- Compare a ranges solution with a traditional algorithm solution.

---

## Phase 6: Reliability, tests, and diagnostics

Professional C++ work requires evidence that code is correct and tools that
find mistakes humans miss.

| Subject | Status | Material |
| --- | --- | --- |
| Assertions, exceptions, and error boundaries | Missing | Explain when to throw and when not to. |
| Error values and `std::expected` | Missing | Cover C++23 `std::expected`. |
| Unit tests and fixtures | Available | [GoogleTest](../testing/gtest/index.md) |
| Test design, integration tests, and fakes | Missing | Focus on behavior and failure paths. |
| Undefined behavior | Missing | Cover lifetime, bounds, overflow, races, and invalid shifts. |
| Address/Undefined/Thread sanitizers | Missing | Add compiler flags and failure examples. |
| Static analysis and formatting | Missing | Add clang-tidy and clang-format. |
| Structured logging | Available | [spdlog](../libraries/log/spdlog/index.md) |

### Mission 6: Harden the address book

Turn Mission 3 into a tested and diagnosable application.

Completion criteria:

- Unit-test parsing, lookup, and error cases with GoogleTest.
- Run AddressSanitizer and UndefinedBehaviorSanitizer.
- Add useful logs without logging every line of code.
- Fuzz or repeatedly test malformed input.
- Produce a short bug report for one defect found by a tool.

---

## Phase 7: Concurrency

Concurrency is not “start a thread.” It is shared-state design, synchronization,
cancellation, lifetime, and the C++ memory model.

| Subject | Status | Material |
| --- | --- | --- |
| Threads, mutexes, and condition variables | Partial | [Multithreading](multithread/index.md) |
| `lock_guard`, `unique_lock`, and scoped locking | Partial | Extend the current mutex examples. |
| Futures, promises, and `async` | Missing | Teach result and exception transport. |
| `jthread`, `stop_token`, and cancellation | Missing | Prefer structured thread lifetime. |
| Atomics and memory ordering | Missing | Begin with default sequential consistency. |
| Deadlock, starvation, and contention | Missing | Include lock ordering and measurement. |
| Thread-safe queues and worker pools | Missing | Build from condition variables. |
| Parallel algorithms | Missing | Explain when parallel execution is safe. |

### Mission 7: Concurrent telemetry pipeline

Create producers that generate measurements and one consumer that writes them.

Completion criteria:

- Use a bounded thread-safe queue.
- Define what happens when the queue is full.
- Stop cleanly without detached threads.
- Preserve or explicitly relax record ordering.
- Run ThreadSanitizer and explain its result.
- Measure throughput and latency before optimizing.

---

## Phase 8: Application engineering and capstone

Combine language knowledge with libraries, configuration, packaging, and
operational behavior.

| Subject | Status | Material |
| --- | --- | --- |
| Command-line interfaces | Available | [CLI11](../libraries/cli11/index.md) |
| YAML configuration | Available | [yaml-cpp](../libraries/yaml_cpp/index.md) |
| Formatting | Available | [{fmt}](../libraries/fmt/index.md) |
| Logging and CSV telemetry | Available | [spdlog](../libraries/log/spdlog/index.md) |
| Serialization | Available | [MessagePack](../libraries/msgpack/index.md) |
| Dependency management and installation | Missing | Cover CMake install/export and package managers. |
| API/ABI compatibility | Missing | Important for shared libraries. |
| Benchmarking and profiling | Missing | Measure CPU, allocation, I/O, and contention. |
| Networking and asynchronous I/O | Missing | Add after concurrency fundamentals. |
| Coroutines and modules | Missing | Advanced; learn when a project needs them. |

### Mission 8: Production-style robotics service

Build a configurable telemetry service that joins the previous phases.

Required features:

- CLI11 subcommands such as `run`, `validate-config`, and `version`.
- yaml-cpp configuration mapped into validated C++ structures.
- Named spdlog component loggers and a telemetry output.
- MessagePack serialization for one message type.
- A bounded worker queue with clean cancellation.
- Unit and integration tests.
- Sanitizer and clang-tidy runs in the build workflow.
- Installation through CMake and a short operator README.
- A benchmark with measurements and one justified optimization.

The capstone is complete only when another person can clone, build, test, run,
and diagnose it from the documentation.

---

## Recommended study routine

For each subject:

1. Read one focused lesson.
2. Reproduce its example without copy/paste.
3. Change the example until it fails, then explain the failure.
4. Write a smaller example from memory.
5. Apply the idea to the current mission.
6. Add tests and commit the result.
7. Review the code one week later and simplify it.

Aim for consistent deliberate practice rather than rushing through headings.
Mastery means you can explain trade-offs, diagnose failures, and design a
maintainable solution—not that you memorized every library function.

## Primary references

- [C++ language and standard library reference](https://en.cppreference.com/w/cpp){:target="_blank" rel="noopener noreferrer"}
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines){:target="_blank" rel="noopener noreferrer"}
- [CMake tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html){:target="_blank" rel="noopener noreferrer"}

<!-- post-content-skill: 1.0.0 -->
