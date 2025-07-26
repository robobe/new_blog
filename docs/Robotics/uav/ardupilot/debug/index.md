---
title: Debug SITL
tags:
    - ardupilot
    - debug
---

{{ page_folder_links() }}

Debug ardupilot SITL using vscode

[Debugging with GDB using VSCode](https://ardupilot.org/dev/docs/debugging-with-gdb-using-vscode.html)

## Debug steps
- Build SITL with debug symbols



```bash
./waf configure --debug
```

or

```bash
sim_vehicle.py -v ArduCopter -f quad --console --map -D
```


```bash title="check if build with debug symbols"
# with
file arducopter 
#
arducopter: ELF 64-bit LSB pie executable, x86-64, version 1 (SYSV), dynamically linked, interpreter /lib64/ld-linux-x86-64.so.2, BuildID[sha1]=9d5c29c3cd71ea0de53ee3585adc661d1650e8f8, for GNU/Linux 3.2.0, with debug_info, not stripped

# without (download precompiled from firmware.ardupilot.org)
file arducopter 
#
arducopter: ELF 64-bit LSB executable, x86-64, version 1 (GNU/Linux), statically linked, BuildID[sha1]=2a762be075efcc4fe2b39bf1ce56873b3aa33a6d, for GNU/Linux 3.2.0, not stripped

```