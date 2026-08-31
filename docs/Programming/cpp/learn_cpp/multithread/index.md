---
title: Modern C++ multithreading
tags:
    - cpp
    - modern
    - threading
---

| Stage | Topic                  | Key C++ APIs                         | Hands-on                        |
| ----- | ---------------------- | ------------------------------------ | ------------------------------- |
| 1     | Thread basics          | `std::jthread`, `std::thread`        | Run two workers                 |
| 2     | Thread lifetime        | RAII, `join`, `stop_token`           | Stoppable worker                |
| 3     | Data races             | shared state                         | Intentionally create/fix a race |
| 4     | Mutexes                | `mutex`, `lock_guard`, `scoped_lock` | Safe counter                    |
| 5     | Condition variables    | `condition_variable`                 | Producer/consumer               |
| 6     | Atomics                | `atomic`, memory ordering basics     | Atomic counter                  |
| 7     | Tasks & results        | `future`, `promise`, `async`         | Parallel calculation            |
| 8     | C++20 coordination     | semaphore, latch, barrier            | Worker synchronization          |
| 9     | Thread-safe structures | queues, ownership                    | Blocking queue                  |
| 10    | Thread pools           | work queues, workers                 | Build small pool                |
| 11    | Advanced atomics       | memory model                         | SPSC queue                      |
| 12    | Architecture           | message passing, executors           | Robotics pipeline               |
