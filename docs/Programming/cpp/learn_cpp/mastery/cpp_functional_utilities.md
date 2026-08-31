# C++ functional utilities

## Summary

Functional utilities let C++ programs pass behavior as data. This lesson progresses from lambdas and callable objects to STL algorithms, composition, and reusable higher-order functions.

## Foundations

- **Lambda basics** — Define a small anonymous function at the point where it is used.
- **Capturing variables** — Give a lambda access to surrounding values by copy or reference.
- **Introduction to `std::function`** — Store and invoke different compatible callable types through one interface.
- **Callable objects** — Create objects that behave like functions by defining `operator()`.

## STL algorithms and predicates

- **Predicates in the STL** — Supply a callable that tests a condition and returns `bool`.
- **Algorithm parameters** — Customize algorithms such as `std::sort`, `std::find_if`, and `std::transform` with callables.
- **Stateful lambdas** — Preserve captured state across calls, for example to count or accumulate results.
- **Function composition** — Combine small callables so the output of one becomes the input of another.

## Advanced techniques

- **Higher-order functions** — Write functions that accept, return, or create other callables.
- **Returning lambdas** — Build a callable configured by values supplied to a factory function.
- **Generic lambdas** — Use `auto` parameters so one lambda works with multiple compatible types.
- **Partial application** — Fix some arguments now and return a callable that receives the remaining arguments later.
