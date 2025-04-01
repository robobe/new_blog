---
tags:
    - ros
    - diagnostics
    - monitor
---

# Diagnostic Monitor

## RQT

```bash title="install"
sudo apt install ros-humble-rqt-robot-monitor
sudo apt install ros-humble-rqt-runtime-monitor
```

### robot monitor

subscribe to:

- /diagnostics_agg
 
### robot runtime monitor
Show stale message

!!! note "stale"
    In diagnostic_aggregator, stale severity is the severity level assigned to a diagnostic status when a message is not received within the configured timeout. This helps in detecting missing diagnostics.
     
subscribe to:

- /diagnostics
- /diagnostics_agg