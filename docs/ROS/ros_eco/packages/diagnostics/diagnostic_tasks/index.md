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