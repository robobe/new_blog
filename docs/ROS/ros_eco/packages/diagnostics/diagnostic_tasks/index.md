---
tags:
    - ros
    - diagnostic
    - tasks
---

# Diagnostic Tasks

DiagnosticTask is an abstract base class for collecting diagnostic data. 

A DiagnosticTask has a name, and a function that is called to create a DiagnosticStatusWrapper. 

DiagnosticsTask subclass by

- CompositeDiagnosticTask
- FrequencyStatus
- GenericFunctionDiagnosticTask
- Heartbeat
- TimeStampStatus


## DiagnosticStatus as a function
Implement DiagnosticStatus as function that register to DiagnosticUpdater

<details>
    <summary>Demo code</summary>

```python
--8<-- "docs/ROS/ros_eco/packages/diagnostics/diagnostic_tasks/code/diagnostic_statis_function_demo.py"
```
</details>



- dummy_diagnostic method get one argument stat of type diagnostic_updater.DiagnosticStatusWrapper DiagnosticStatusWrapper is a derived class of diagnostic_msgs.msg.DiagnosticStatus that provides a set of convenience methods.

- diagnostic_updater use `add` method to register diagnostic method