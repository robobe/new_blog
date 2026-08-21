---
title: C++ Testing Learning Path
tags:
    - cpp
    - testing
    - learning-path
---

# C++ Testing Learning Path

This note records possible topics for future tutorials and discussion. The
priority is learning how to design valuable tests and testable C++ modules,
not only learning more testing-framework macros.

## 1. Behavior-focused test design

Learn the Arrange, Act, Assert pattern and test observable behavior through a
module's interface rather than depending on private implementation details.

For each behavior, consider:

- the normal case
- boundary values
- invalid input
- failure paths
- state transitions

## 2. Designing testable modules

Practice these design principles:

- Accept dependencies instead of constructing them internally.
- Return results instead of hiding behavior in side effects.
- Put a seam around hardware, time, files, networking, and databases.
- Keep the public interface small and place complexity in the implementation.
- Provide a real adapter for production and a fake adapter for tests when the
  dependency genuinely varies.

Tests and callers should use the same module interface. If a test must reach
past that interface, reconsider where the seam belongs.

## 3. Unit and integration tests

Learn when each test level is appropriate:

| Test level | Purpose |
|---|---|
| Unit test | Exercise one module through its interface with controlled dependencies. |
| Integration test | Verify real adapters and multiple modules working together. |

Prefer small state-based fakes over mocking every class. Use integration tests
for behavior that only becomes meaningful when real dependencies interact.

## 4. Catch2 sections and data generators

After the test-design fundamentals, learn:

- `SECTION` for independent paths that share setup
- nested sections and their execution model
- `GENERATE` for testing multiple representative values
- tags and command-line filtering

Catch2 runs a test case from the beginning for each section path, so each path
receives fresh local state.

## 5. Fixtures, matchers, and test doubles

Add these tools when simpler tests are no longer sufficient:

- fixtures for shared setup with a meaningful lifetime
- matchers for expressive checks of complex values
- fakes for stateful external dependencies
- stubs for fixed dependency responses
- mocks only when verifying an interaction is the required behavior

## Suggested practical tutorial

Build a testable embedded-style `TemperatureMonitor` module:

```text
TemperatureMonitor
        |
        | reads through an interface
        v
TemperatureSensor
   +-- HardwareSensor adapter
   `-- FakeSensor adapter
```

Test these behaviors:

- A temperature below the threshold produces no alarm.
- Crossing the threshold activates the alarm.
- A sensor error is reported correctly.
- Repeated readings do not create duplicate alarms.
- Boundary values at the threshold follow the documented rule.

This example introduces dependency injection, a real seam, production and fake
adapters, failure paths, and the difference between unit and integration tests.

## Future topics

1. Arrange, Act, Assert and behavior-oriented test names
2. Boundary-value and equivalence-partition testing
3. Testable module design and dependency injection
4. Fakes, stubs, and mocks
5. Unit versus integration testing
6. Catch2 `SECTION` and `GENERATE`
7. Fixtures and custom matchers
8. Property-based testing
9. Sanitizers and tests
10. Continuous integration and test reporting

## References

- [Catch2 test cases and sections](https://github.com/catchorg/Catch2/blob/devel/docs/test-cases-and-sections.md)
- [Catch2 tutorial](https://github.com/catchorg/Catch2/blob/devel/docs/tutorial.md)
