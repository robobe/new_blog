---
title: C++ Mastery Curriculum Plan
tags:
    - cpp
    - mastery
    - planning
---

## Goal

Create a dependency-ordered C++ curriculum under `learn_cpp/mastery/`. Existing
lessons remain unchanged and are linked as supporting material. Core lessons use
C++20 and GCC 13. The `std::expected` example is the only C++23 target. Modules
remain conceptual.

## Lesson contract

Every runnable lesson contains:

- 15–25 minutes of focused instruction;
- 2–4 runnable examples;
- a local `code/CMakeLists.txt` using warnings and CTest;
- a compiling `exercise_starter.cpp`;
- predict-output, concept, find-the-bug, and coding-exercise questions;
- collapsible answers explaining the reasoning;
- prerequisites and completion criteria.

Lesson exercises include solutions. Phase missions provide requirements,
hints, and acceptance tests but no complete solution.

## Phase 1 — Tools and foundations

1. Toolchain and debugging: preprocessing, compiler, assembler, linker, loader,
   translation units, headers, namespaces, ODR, GDB, and VS Code.
2. Values and expressions: fundamental types, initialization, conversions,
   signed/unsigned behavior, operators, precedence, and short-circuiting.
3. Control flow and functions: conditions, loops, scope, functions, return
   values, and overloads.
4. Text and data modeling: `string`, `string_view`, `enum class`, structures,
   and simple invariants.

Mission: build and debug a warning-free multi-file command-line text analyzer.

## Phase 2 — STL mastery

5. Iterators and algorithms.
6. Containers, ordering, hashing, complexity, and invalidation.
7. Vocabulary types: `optional`, `variant`, `tuple`, and `span`.
8. `filesystem`, `chrono`, random engines, and distributions.

Mission: build a portable file-backed address book using standard algorithms
and associative containers.

## Phase 3 — Reliability and design

9. Assertions, exceptions, error boundaries, explicit errors, and a marked
   C++23 `std::expected` example.
10. Unit/integration tests, fixtures, fakes, and failure paths.
11. Undefined behavior, sanitizers, clang-tidy, and clang-format.
12. Composition, dependency injection, invariants, strong types, and practical
    SOLID principles.

Mission: harden the address book with tests, tools, logging, and fake storage.

## Phase 4 — Generic programming

13. `constexpr`, `consteval`, `constinit`, type traits, concepts, and constraints.
14. Variadic templates, forwarding, and fold expressions.
15. Ranges, views, projections, lazy evaluation, and lifetime hazards.

Mission: build a constrained generic statistics library.

## Phase 5 — Concurrency

16. Futures, promises, `async`, `jthread`, `stop_token`, and cancellation.
17. Atomics, memory ordering, locking, deadlock, starvation, and contention.
18. Bounded queues, worker pools, backpressure, shutdown, and parallel
    algorithms.

Mission: build and measure a bounded concurrent telemetry pipeline.

## Phase 6 — Production engineering

19. Dependency management, CMake install/export/package configuration, and
    API/ABI compatibility.
20. Benchmarking and CPU, allocation, I/O, and contention profiling.

Mission: package a reusable library, consume it externally, benchmark it, and
document one measured optimization.

## Specializations

21. Pure C++20 coroutines: coroutine frames, `promise_type`, `co_await`,
    `co_yield`, `co_return`, a learning `Generator<T>`, and `Task<T>`.
22. Conceptual C++20 modules: interfaces, `export module`, `import`, and the
    difference from headers. No GCC 13 runnable project is required.

Networking is excluded until an asynchronous runtime library is selected.

## Verification

- Build every normal example with GCC 13 in C++20 mode.
- Enable `-Wall -Wextra -Wpedantic` and CTest from the first lesson.
- Compile phase missions with GCC 13 and Clang 18.
- Test deterministic output plus invalid and failure paths.
- Introduce sanitizers in Phase 3 and ThreadSanitizer in Phase 5.
- Build MkDocs after every lesson and reject new page-specific warnings.
- Change syllabus status only after lesson examples, quiz, exercise, tests, and
  documentation validation are complete.

<!-- post-content-skill: 1.0.0 -->
