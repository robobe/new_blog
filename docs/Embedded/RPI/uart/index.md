---
title: RPI5 uart ports on gpio
tags:
    - gpio
    - uart
    - rpi5
---

sudo apt remove modemmanager


crw--w---- 1 root tty 204, 64 May 12 20:54 /dev/ttyAMA0

```
cat /proc/cmdline 
reboot=w coherent_pool=1M 8250.nr_uarts=1 pci=pcie_bus_safe  smsc95xx.macaddr=2C:CF:67:F6:3B:25 vc_mem.mem_base=0x3fc00000 vc_mem.mem_size=0x40000000  console=ttyAMA0,115200 multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=IL
```

```
console=serial0,115200 multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=IL
```

remove

```
console=serial0,115200  
```

## After boot
```
crw-rw---- 1 root dialout 204, 64 May 12 20:58 /dev/ttyAMA0

```

## Check with echo

```
socat -x -v /dev/ttyAMA0,raw,echo=0,b115200 -
```

```python title="echo"
import serial
import time

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)

for i in range(10):
    message = f"hello {i}\n"

    print(f"TX: {message.strip()}")
    ser.write(message.encode())

    response = ser.readline().decode().strip()
    print(f"RX: {response}")

    time.sleep(0.5)

ser.close()
```