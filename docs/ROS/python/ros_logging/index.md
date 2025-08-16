---
tags:
    - ros
    - python
    - logging
    - throttle_duration_sec
---


{{ page_folder_links() }}


```
self.get_logger().info(f'IMU Orientation: Pitch={p} <==>  cmd={self.pitch}', throttle_duration_sec=1.0)
```