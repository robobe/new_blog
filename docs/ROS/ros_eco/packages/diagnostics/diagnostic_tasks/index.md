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
- [Heartbeat](#heartbeat)
- TimeStampStatus


## DiagnosticStatus as a function
Implement DiagnosticStatus as function that register to DiagnosticUpdater

<details>
    <summary>Demo code</summary>

```python
--8<-- "docs/ROS/ros_eco/packages/diagnostics/diagnostic_tasks/code/diagnostic_status_function_demo.py"
```
</details>



- dummy_diagnostic method get one argument stat of type diagnostic_updater.DiagnosticStatusWrapper DiagnosticStatusWrapper is a derived class of diagnostic_msgs.msg.DiagnosticStatus that provides a set of convenience methods.

- diagnostic_updater use `add` method to register diagnostic method
- The updater publish diagnostic message every one second

<details>
    <summary>/diagnostics topic</summary>

```yaml title="ros2 topic echo /diagnostics"
---
header:
  stamp:
    sec: 1743564548
    nanosec: 360259990
  frame_id: ''
status:
- level: "\x01"
  name: 'minimal: dummy_diagnostic'
  message: message dummy_diagnostic
  hardware_id: ''
  values: []
---
header:
  stamp:
    sec: 1743564549
    nanosec: 360183120
  frame_id: ''
status:
- level: "\x01"
  name: 'minimal: dummy_diagnostic'
  message: message dummy_diagnostic
  hardware_id: ''
  values: []
```
</details>

---

## Heartbeat
Diagnostic task to monitor whether a node is alive. This diagnostic task always reports as **OK** and **'Alive'** when it runs

<details>
    <summary>Demo code</summary>

```python
--8<-- "docs/ROS/ros_eco/packages/diagnostics/diagnostic_tasks/code/diagnostic_heartbeat_demo.py"
```
</details>


<details>
    <summary>/diagnostic topic</summary>

```yaml
---
header:
  stamp:
    sec: 1743565018
    nanosec: 240255516
  frame_id: ''
status:
- level: "\0"
  name: 'node_name: Heartbeat'
  message: Alive
  hardware_id: hwid
  values: []
---
header:
  stamp:
    sec: 1743565019
    nanosec: 239948956
  frame_id: ''
status:
- level: "\0"
  name: 'node_name: Heartbeat'
  message: Alive
  hardware_id: hwid
  values: []
```
</details>

