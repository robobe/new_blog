---
title: FSM finite state machine
tags:
    - state machine
    - fsm
---

A Finite State Machine (FSM) is a computational model used to design systems that can exist in only one of a set number of "states" at any given time. It transitions from one state to another in response to specific inputs or events

<div class="grid-container">
    <div class="grid-item">
        <a href="#transitions">
                <p>transitions</p>
        </a>
    </div>
</div>

---

##  transitions
A lightweight, object-oriented state machine implementation in Python with many extensions [github](https://github.com/pytransitions/transitions)

```bash
pip install transitions
```

### Demo: simple example

Very simple example that move from state to state without condition every time that trigger called

The demo attaches a `Machine` to an empty `Robot` object, starts it in the `IDLE` state, and registers three transitions:
`start_tracking` moves from `IDLE` to `TRACKING`, `error` moves from any state to `ERROR`, and `reset` moves from `ERROR` back to `IDLE`.
It prints the robot state after each trigger, showing the sequence `IDLE`, `TRACKING`, `ERROR`, `IDLE`.

<details>
<summary>simple demo</summary>

```
--8<-- "docs/Robotics/algorithms/fsm/code/basic_example.py"
```
</details>


### Demo: simple example II

This demo defines the states with a Python `Enum`, which keeps the state names grouped and avoids repeating raw strings throughout the code.
The transitions are the same basic flow as the first example: `start_tracking` moves from `IDLE` to `TRACKING`, `error` can move from any state to `ERROR`, and `reset` moves from `ERROR` back to `IDLE`.

The `Robot` class also defines `on_enter_ERROR` and `on_exit_ERROR`, so the `transitions` library automatically calls those methods when the robot enters or leaves the `ERROR` state.
After triggering `error`, the demo uses the generated `robot.is_ERROR()` helper to check the current state, then calls `reset`.
At the end it calls the generated `robot.to_ERROR()` method to move directly into `ERROR` without using a custom trigger.

<details>
<summary>more simple demo</summary>

```
--8<-- "docs/Robotics/algorithms/fsm/code/basic_example_II.py"
```
</details>


### Demo: Conditions

This demo shows how to keep transition logic inside named condition methods instead of scattering `if` statements around the application.
The robot stores sensor and system data in a `Context` dataclass, then several transitions share the same `resolve` trigger.
When `resolve()` is called, the machine checks the valid transition for the current state and only moves if its condition returns `True`.

`IDLE` moves to `SEARCH` only when the camera is connected, the battery is above `10.5`, and there is no active error.
`SEARCH` moves to `TRACKING` when a target is found with confidence greater than `0.75`.
`TRACKING` moves to `RECOVERY` when the target is lost but the retry count is still below `3`.
The wildcard transition from `*` to `ERROR` catches critical faults from any state when an error flag is set, the battery drops below `9.5`, or the retry count reaches `3`.

The script updates the context before each `resolve()` call, so the printed states show the conditional path `SEARCH`, `TRACKING`, then `RECOVERY`.

<details>
<summary>State Machine with conditions</summary>
```
--8<-- "docs/Robotics/algorithms/fsm/code/conditions.py"
```
</details>

### Demo: Conditions with state change callback

This example uses the same conditional state machine, but adds a callback that runs after every successful state change.
The machine is created with `send_event=True`, so callbacks receive an event object with transition metadata.
`after_state_change=self.on_state_changed` registers one callback for all transitions, and `event.transition.source` / `event.transition.dest` provide the previous and new state.

Because `send_event=True` also passes the event object to condition callbacks, the condition methods accept an `event` argument even when they do not use it.

<details>
<summary>State Machine with conditions and callback</summary>

```
--8<-- "docs/Robotics/algorithms/fsm/code/conditions_with_callback.py"
```
</details>


The idea is to create main loop the update the context and resolve the state machine if two or more state has matching condition the first one return `declare order matter"

```python
while True:
    update_context()
    fsm.resolve()
```

---

More `add_transition` arguments

- **conditions**: Runs guard functions. The transition happens only if all return **True**.

- **unless**: Opposite of conditions. The transition happens only if all unless callbacks return **False**.

- **prepare**: Runs before condition checks. Use it to update or normalize data needed by guards.

- **before**: Runs after the transition is approved, but before the state actually changes. Use it for actions that should happen only when the transition is definitely going to occur.

- **after**: Runs after the state has changed. Use it for actions that depend on the new state already being active.


---

### DEMO: Add callback when state changed

This example is based on the conditional FSM, but the `Machine` also registers a global state-change callback:

```python
after_state_change=self.on_state_changed
```

`after_state_change` runs after every successful transition, no matter which trigger caused it.
In this example, every transition uses the same `resolve()` trigger, so the callback runs after `IDLE -> SEARCH`, `SEARCH -> TRACKING`, and `TRACKING -> RECOVERY`.
It does not run when `resolve()` is called but no transition condition matches, because the state did not change.

The machine is also created with `send_event=True`.
This tells `transitions` to pass one event object into callbacks instead of calling them with no arguments.
The callback uses that event object to read the transition that just happened:

```python
def on_state_changed(self, event):
    previous_state = event.transition.source
    new_state = event.transition.dest
    print(f"State changed: {previous_state} -> {new_state}")
```

`event.transition.source` is the previous state and `event.transition.dest` is the new state.
That makes this callback useful for logging, publishing state changes, updating a UI, or sending diagnostics whenever the robot changes mode.

Because `send_event=True` affects all transition callbacks, the condition methods also receive the event object:

```python
def can_start_search(self, event):
```

The condition methods in this example do not need the event, but they still accept it so their signatures match the way the machine calls them.

<details>
<summary>code</summary>
```
--8<-- "docs/Robotics/algorithms/fsm/code/machine_callback.py"
```
</details>
