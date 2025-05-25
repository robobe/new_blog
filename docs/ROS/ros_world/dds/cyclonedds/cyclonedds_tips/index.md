---
tags:
    - dds
    - cyclonedds
    - tips
---

# Cyclonedds 


## Send large message like Image

[stereolabs.com: DDS Middleware and Network tuning](https://www.stereolabs.com/docs/ros2/dds_and_network_tuning)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <Internal>
      <SocketReceiveBufferSize min="20MB"></SocketReceiveBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
```

```
sudo sysctl -w net.core.rmem_max=30000000
sudo sysctl -w net.core.rmem_max=2147483647 
```