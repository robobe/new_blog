---
title: Python command scheduler
tags:
    - python
    - scheduler
    - threading
    - command-pattern
---

This mini project implements a small background scheduler for Python commands.
It can run a command once, run a command repeatedly, cancel a repeated command by
key, and store execution errors in a shared context.

The code lives under `scheduler_pkg` and is split into small modules:

| File | Responsibility |
| --- | --- |
| `commands.py` | Defines the command interface and the wrapper used for repeated commands. |
| `scheduler_queue.py` | Stores future commands in time order. |
| `cancellation.py` | Tracks which keyed commands are still active. |
| `worker.py` | Runs the scheduler loop in the background thread. |
| `scheduler.py` | Provides the public `CommandScheduler` API. |
| `usage_example.py` | Shows how to create commands and run the scheduler. |
| `exception_handling_example.py` | Shows how scheduler errors can be captured or reported. |

## Basic Idea

The scheduler keeps pending commands in a priority queue built with Python's
`heapq` module. When new work is submitted, `heapq.heappush` inserts a
`QueuedCommand` into the heap. Because `QueuedCommand` is ordered by `run_at`
first and `sequence` second, the command with the earliest scheduled time stays
at the front of the queue. If two commands have the same `run_at` value,
`sequence` preserves the order they were added.

The worker can then look at the first heap item to know which command has the
highest priority to run next. It does not need to sort the whole queue each time
a command is added.

The scheduler is based on the command pattern. A task is represented by a class
that inherits from `Command` and implements `execute`.

```python
@dataclass
class PrintCommand(Command):
    message: str

    def execute(self, context: SchedulerContext) -> None:
        print(self.message)
```

The scheduler does not need to know what the command does. It only needs to know
when the command should run and whether it should run again later.

There are two main ways to add work:

```python
scheduler.submit(PrintCommand("runs once"))
```

`submit` runs a command once, after an optional delay.

```python
scheduler.schedule(
    PrintCommand("runs repeatedly"),
    interval_s=0.5,
    key="printer",
)
```

`schedule` wraps the command in a `ScheduledCommand`, gives it a repeat interval,
and requeues it after each successful attempt while it is still active.



## Main Classes

### Command

`Command` is the base abstraction for work.

```python
class Command(ABC):
    key: ClassVar[str | None] = None
    repeat_interval_s: float | None = None

    @abstractmethod
    def execute(self, scheduler: SchedulerContext) -> Any:
        pass
```

Every command must implement `execute`. The scheduler passes a context object to
that method, so commands can read or update shared application state.

The optional `key` is used for cancellation. Commands without a key are always
considered active.

### ScheduledCommand

`ScheduledCommand` wraps another command and adds repeat behavior.

```python
ScheduledCommand(
    command=command,
    repeat_interval_s=interval_s,
    key_override=key,
)
```

When its `execute` method runs, it simply delegates to the original command. This
keeps the repeat logic outside the command itself.

### TimedCommandQueue

`TimedCommandQueue` stores commands in a heap. A heap is useful here because the
worker always needs the command with the earliest `run_at` time.

Each queued item contains:

- `run_at`: when the command should run
- `sequence`: a tie breaker for commands scheduled at the same time
- `token`: the cancellation token
- `command`: the command object

`pop_ready` only returns commands whose `run_at` time has passed and whose token
is still active.

### CancellationRegistry

The cancellation system uses tokens instead of removing old items from the heap.

When a keyed command is submitted, the registry creates a new object token and
stores it under that key. When `remove(key)` is called, the registry replaces the
stored token with a new object.

Old queued commands still exist in the heap, but they no longer match the active
token. The worker skips them when they reach the front of the queue.

This keeps cancellation simple because the queue does not need to search through
all pending items.

### SchedulerWorker

`SchedulerWorker` owns the run loop:

1. Check whether a command is ready.
2. If nothing is ready, wait until the next command time or until a wake event.
3. Execute the command.
4. Store or report errors.
5. Requeue repeating commands that are still active.

The worker uses:

- a `Lock` to protect shared queue and cancellation data
- a `wake_event` so new commands or removals can interrupt waiting
- a `stop_event` so `stop()` can end the loop

### CommandScheduler

`CommandScheduler` is the public API that connects all the lower-level objects.

```python
scheduler = CommandScheduler(context)
scheduler.start()
scheduler.submit(command)
scheduler.schedule(command, interval_s=1.0, key="job")
scheduler.remove("job")
scheduler.stop()
```

`start` creates a daemon thread and runs the worker loop inside it. `stop` sets
the stop event, wakes the worker, and joins the thread.

## Example Flow

The example in `usage_example.py` creates an application context and a
`PrintCommand`.

```python
context = AppContext()
scheduler = CommandScheduler(context)
scheduler.start()

try:
    scheduler.submit(PrintCommand("runs once"))
    scheduler.schedule(PrintCommand("runs repeatedly"), interval_s=0.5, key="printer")
    time.sleep(2.0)
    scheduler.remove("printer")
finally:
    scheduler.stop()
```

The first command runs once. The second command runs every half second until
`remove("printer")` invalidates its cancellation token. Finally, `stop()` shuts
down the worker thread.

## Why The Design Works

The scheduler separates responsibilities:

- commands define what to do
- the queue defines when to run
- the cancellation registry defines whether a keyed command is still valid
- the worker defines the background execution loop
- the scheduler class gives users a small public API

This makes the project easy to extend. For example, new command classes can be
added without changing the scheduler, and different error handling can be added
by passing an `on_error` callback to `CommandScheduler`.


---

## Source Code

<details>
<summary>__init__.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/__init__.py"
```

</details>

<details>
<summary>commands.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/commands.py"
```

</details>

<details>
<summary>cancellation.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/cancellation.py"
```

</details>

<details>
<summary>scheduler_queue.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/scheduler_queue.py"
```

</details>

<details>
<summary>worker.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/worker.py"
```

</details>

<details>
<summary>scheduler.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/scheduler.py"
```

</details>

## Entry Point

<details>
<summary>usage_example.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/usage_example.py"
```

</details>

<details>
<summary>exception_handling_example.py</summary>

```python
--8<-- "docs/Programming/python/mini_projects/scheduler/scheduler_pkg/exception_handling_example.py"
```

</details>
