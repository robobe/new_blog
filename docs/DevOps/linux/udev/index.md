---
tags:
    - udev
    - linux
    - custom
    - rules
---

# Custom udev rules

![](../../../assets/images/under_construction.png)

udev (the device manager for the Linux kernel) uses rules to manage device events (e.g., when a USB is plugged in). Rules are typically stored in `/etc/udev/rules.d/` or `/lib/udev/rules.d/` as files with a `.rules` extension (e.g., 10-my-rule.rules). Lower-numbered files are processed first.

## udev Rule
A rule consists of match keys (conditions) and assignment keys (actions), written in the format:

```
KEY=="value", ACTION="value"
```

- Match keys: **Conditions** to identify a device (e.g., SUBSYSTEM, ATTR).
- Assignment keys: **Actions** to take (e.g., NAME, RUN, SYMLINK).


## demos
### Set symbolic name
Set symbolic name every time my arduino mega connected



```bash
udevadm info -a -p /sys/class/tty/ttyACM0
```

- **udevadm**: A utility for managing and querying udev, the Linux device manager.
- **info**: A subcommand that provides information about a device.
- **-a** : Walks up the device tree (from the specified device to its parents) and shows all attributes available for matching in udev rules.
- **-p** : Specifies the sysfs path of the device to query `/sys/class/tty/ttyACM0`.

<details>
    <summary>result: Show only the part i use for the rule</summary>

```
looking at parent device '/devices/pci0000:00/0000:00:14.0/usb3/3-4/3-4.1/3-4.1.5/3-4.1.5.3':
    KERNELS=="3-4.1.5.3"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{authorized}=="1"
    ...
    ATTRS{configuration}==""
    ATTRS{devnum}=="24"
    ATTRS{devpath}=="4.1.5.3"
    ATTRS{idProduct}=="0010"
    ATTRS{idVendor}=="2341"
    ATTRS{ltm_capable}=="no"
    ATTRS{manufacturer}=="Arduino (www.arduino.cc)"
    ...
    ATTRS{product}=="Arduino Mega 2560"
    ATTRS{quirks}=="0x0"

```
</details>



```bash title="/etc/udev/rules.d/91-arduino.rules"
ACTION=="add",SUBSYSTEMS=="usb",ATTRS{idVendor}=="2341",ATTRS{product}=="Arduino Mega 2560", SYMLINK+="arduino_mega"
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

- Disconnect / Connect the device for `add` event
```bash
ll /dev/arduino_mega 
#
lrwxrwxrwx 1 root root 7 Apr  2 06:56 /dev/arduino_mega -> ttyACM0
```


---
# TODO
- explain monitor
- explain device tree
- explain ATTR ATTRS ENV
- explain SUBSYSTEMS
  
---

### Set permission
Set permission to usb device

```bash
SUBSYSTEMS=="i2c-dev", ATTRS{name}=="CP2112 SMBus Bridge on hidraw5", MODE="0666"
```

