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
export CYCLONEDDS_URI="file://$HOME/CycloneDDS/my-config.xml"
```

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