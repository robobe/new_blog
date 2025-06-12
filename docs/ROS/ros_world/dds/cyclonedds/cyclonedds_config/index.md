---
tags:
    - dds
    - cyclonedds
    - config
    - cyclonedds_uri
---

# CycloneDDS configuration
Control cyclonedds discovery, network and tracing settings using xml file  
[more](https://cyclonedds.io/docs/cyclonedds/latest/config/index.html)

!!! note "cyclone version"
     Check to right documentation for installed version


```bash title="usage"
export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
```

<div class="grid-container">
     <div class="grid-item">
            <a href="unicast">
                <p>unicast</p>
            </a>
        </div>
        <div class="grid-item">
             <a href="#">
                <p>TBD</p>
            </a>
        </div>
    <div class="grid-item">
          <a href="">
                <p>TBD</p>
            </a>
    </div>
    <div class="grid-item">
            <a href="#reporting-and-tracing">
                <p>Tracing</p>
            </a>
        </div>
        <div class="grid-item">
             <a href="#shm">
                <p>shm</p>
            </a>
        </div>
    <div class="grid-item">
          <a href="cyclonedds_tips">
                <p>large messages</p>
            </a>
    </div>
</div>

## Reporting and Tracing
[doc](https://cyclonedds.io/docs/cyclonedds/latest/config/reporting-tracing.html)


### Demo: output conf to log file

!!! note "create directory path before running"
     
```xml
<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS
   xmlns="https://cdds.io/config"
   xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
   xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"
 >
   <Domain Id="any">
    <Tracing>
      <Verbosity>config</Verbosity>
      <OutputFile>
        ${HOME}/dds/log/cdds.log.${CYCLONEDDS_PID}
      </OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
```

---

## SHM


[Using Shared Memory with ROS 2](https://github.com/ros2/rmw_cyclonedds/blob/humble/shared_memory_support.md)

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
    <Domain id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>info</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
```

```bash title="run rudi"
iox-roudi
```

---

## Large message

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

```ini title="/etc/sysctl.d/10-cyclone-max.conf"
# IP fragmentation settings
net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB

# Increase the maximum receive buffer size for network packets
net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

```