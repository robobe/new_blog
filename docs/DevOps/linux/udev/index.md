---
tags:
    - udev
    - linux
    - custom
    - rules
---

# Custom udev rules

![](../../../assets/images/under_construction.png)

## write rule
- place `rule` under `/etc/udev/rules.d/xx-my-udev.rule`


## demos
### Set name
Set name / symbolic name every time my arduino mega connected

```bash title="monitor"
udevadm monitor --property --subsystem-match=usb
```
<details>
    <summary>monitor result</summary>

```
...

UDEV  [73077.265181] add      /devices/pci0000:00/0000:00:14.0/usb3/3-4/3-4.1/3-4.1.5/3-4.1.5.4 (usb)
ACTION=add
DEVPATH=/devices/pci0000:00/0000:00:14.0/usb3/3-4/3-4.1/3-4.1.5/3-4.1.5.4
SUBSYSTEM=usb
DEVNAME=/dev/bus/usb/003/019
DEVTYPE=usb_device
PRODUCT=2341/10/1
TYPE=2/0/0
BUSNUM=003
DEVNUM=019
SEQNUM=6445
USEC_INITIALIZED=73077264733
ID_VENDOR=Arduino__www.arduino.cc_
ID_VENDOR_ENC=Arduino\x20\x28www.arduino.cc\x29
ID_VENDOR_ID=2341
ID_MODEL=Arduino_Mega_2560

...
```
</details>


```bash title="/etc/udev/rules.d/91-arduino.rules"
ACTION=="add",SUBSYSTEM=="usb",ATTRS{ID_VENDOR_ID}=="2341",ATTRS{ID_MODEL}=="Arduino_Mega_2560",SYMLINK+="arduino_mega"
```

!!! tip "debug and monitor"
    monitor service using

    ```bash
    sudo journalctl -f -u systemd-udev.service
    
    ```
     

### check

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

### Set permission
Set permission to usb device

```bash
SUBSYSTEMS=="i2c-dev", ATTRS{name}=="CP2112 SMBus Bridge on hidraw5", MODE="0666"
```

