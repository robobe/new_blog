---
tags:
    - ros
    - diagnostic
    - aggregator
---

# Diagnostic aggregator

The Diagnostic Aggregator in ROS 2 is a tool designed to organize and categorize diagnostic messages efficiently. It helps in monitoring system health by grouping diagnostic messages into a structured hierarchy, making it easier to analyze the status of different components. it's subscribe to `/diagnostic` topic and publish to `/diagnostic_agg` topic.

```yaml
diagnostic_aggregator:
  ros__parameters:
    analyzers:
     
      sensors:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "Sensors"
        startswith: ["/sensors"]
        expected: ["/sensors/camera", "/sensors/lidar"]
      
      motors:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "Motors"
        startswith: ["/motors"]
        expected: ["/motors/left_wheel", "/motors/right_wheel"]
      
      system:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "System"
        contains: ["temperature", "battery"]
```

- **path**: Defines the category name in the aggregated output.
- **startswith**: Groups diagnostics messages that start with a certain prefix.
- **expected**: Lists expected diagnostic topics (useful for error checking).
- **contains**: Groups messages that contain a specific word.

### expected

The expected parameter in the aggregator checks for the presence of specific name values inside the messages published to /diagnostics.

!!! note "stale"
    In diagnostic_aggregator, stale severity is the severity level assigned to a diagnostic status when a message is not received within the configured timeout. This helps in detecting missing diagnostics.
     