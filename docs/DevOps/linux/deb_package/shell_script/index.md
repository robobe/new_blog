---
title: Create Debian package for shell script
tags:
    - debian
    - fakeroot
    - rules
    - package
---

- Pack minimal shell script to `debian` package using `fakeroot` 
- Add `postinst` and other debian scripts to control the installation
- Run the script as systemd service


## install helper packages

```bash
sudo apt install fakeroot devscripts debhelper
```

## Tutorial: Minimal 
Check github `simple_script` tag [source code](https://github.com/robobe/debian_tutorial)
### project
```
.
├── .vscode
│   ├── settings.json
│   └── tasks.json
├── debian
│   ├── changelog
│   ├── compat
│   ├── control
│   ├── my-tool.install
│   └── rules
├── debs
├── readme.md
├── .gitignore
└── scripts
    └── my-tool.sh
```



```bash title="scripts/my-tool.sh"
#!/bin/sh

echo "hello my-tool"
```


### debian folder
Using `fakeroot` debian folder must contain this files:

- compat
- rules
- changelog
- control
- my-tool.install


#### my-tools.install
Map a file from your project directory to its destination path in the installed system.
For example, install the script into `/usr/bin` on the target filesystem.

```init title="debian/my-tools.install"
scripts/my-tool.sh /usr/bin
```

!!! tip
    The file name must match the Debian package name.

     

#### changelog
`dch` command create and update changelog file
```bash
# create change log file
dch --create -v 0.0.1 --package my-tool "v 0.0.1"

# add entries
dch -v 0.0.1 "simple script"
```


    
    


#### control

```
Source: my-tools
Section: misc
Priority: optional
Maintainer: Your Name <youremail@example.com>
Standards-Version: 4.6.2

Package: my-tools
Architecture: all
Description: Minimal Debian package
 A short description of the minimal Debian package.
```

!!! Note
    The  `source` section in mandatory by `fakeroot`

#### rules

```make
#!/usr/bin/make -f

%:
	dh $@

override_dh_builddeb:
	dh_builddeb --destdir=$(CURDIR)/debs
```


##### change deb output path (debs folder must be exists)

```make
override_dh_builddeb:
	dh_builddeb --destdir=$(CURDIR)/debs
```

!!! Tip
    The file must use TAB indentation

    Check vscode settings to config rule file as shellscript with TAB indentation

#### compat

```
13
```

---

### build
```bash
fakeroot debian/rules binary
```

### clean
```bash
fakeroot debian/rules clean
```

---

## VSCode

### tasks.json
- **build** using `fakeroot debian/rules binary`
- **clean** `fakeroot debian/rules clean`
- **change log** using `dch`

<details>
    <summary>tasks.json</summary>

```json
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/.vscode/tasks.json"
```
</details>


     

### setting.json
Replace indentation from space to tab when using sheelscript

```json title="settings.json"
    "files.associations": {
        "rules": "shellscript"
    }
```

<details>
    <summary>settings.json</summary>

```json
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/.vscode/settings.json"
```

</details>


!!! tip "using cat to check TAB indentation"
    ```bash
    cat -T <file>
    ```
     

---

### Git

#### .gitignore

```
debs/
debian/*
!debian/changelog
!debian/rules
!debian/control
!debian/compat
!debian/my-tool.install
```

exclude file from ignore folder, for it to work we need to ignore files in folder and not the folder it self `debian/*` 

---

### dpkg 

- `dpkg -c` : List contents of a deb package.
- `dpkg -I` : Show information about a package. 


---

# Debian scripts

Debian packages can include executable scripts that run automatically at different stages of installation, upgrade, and removal. These scripts live in the package’s `debian/` directory. 

!!! Note
    Don't forget to set executable permission

- preinst
- postinst
- prerm
- postrm


## Scripts order

**Install**
1. preinst (install)
1. unpack files
1. postinst (configure)

**Upgrade**
1. preinst (upgrade)
1. unpack new files
1. postinst (configure)

**Remove**
1. prerm (remove)
1. remove files
1. postrm (remove)

**Purge**
1. postrm (purge)


### Check for script existing

```bash
dpkg -I <package name>

#
 new Debian package, version 2.0.
 size 1100 bytes: control archive=472 bytes.
     237 bytes,     9 lines      control
     123 bytes,     2 lines      md5sums
      54 bytes,     7 lines   *  postinst             #!/bin/sh
      52 bytes,     7 lines   *  postrm               #!/bin/sh
```

---

# Pack as systemd service
check source code [github](https://github.com/robobe/debian_tutorial/tree/systemd_service) `systemd_service` tag

```title="services/my-service.service"
[Unit]
Description=mytool background service
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/my-tool.sh
Restart=on-failure
User=user
Group=user

[Install]
WantedBy=multi-user.target

```

```init title="debian/my-tools.install"
scripts/my-tool.sh /usr/bin
services/my-service.service lib/systemd/system/
```

```bash title="debian/postinst"
#!/bin/sh
set -e

systemctl daemon-reload
systemctl enable my-service.service
systemctl start my-service.service

exit 0
```

```bash title="debian/postrm"
#!/bin/sh
set -e

if [ "$1" = "remove" ]; then
    systemctl stop my-service.service || true
fi

if [ "$1" = "purge" ]; then
	echo purge
fi

systemctl daemon-reload || true



exit 0
```


## Install and check

```bash title="journalctl"
journalctl -u my-service.service 
#
Dec 01 08:42:07 omen systemd[1]: Started my-service.service - mytool background service.
Dec 01 08:42:07 omen my-tool.sh[650610]: hello my-tool
Dec 01 08:42:07 omen systemd[1]: my-service.service: Deactivated successfully.
Dec 01 10:07:11 omen systemd[1]: Started my-service.service - mytool background service.
Dec 01 10:07:11 omen my-tool.sh[705043]: hello my-tool
Dec 01 10:07:11 omen systemd[1]: my-service.service: Deactivated successfully.
```

```bash title="systemctl status"
sudo systemctl status my-service.service
# 
○ my-service.service - mytool background service
     Loaded: loaded (/usr/lib/systemd/system/my-service.service; enabled; preset: enabled)
     Active: inactive (dead) since Mon 2025-12-01 10:07:11 IST; 1min 15s ago
   Duration: 1ms
    Process: 705043 ExecStart=/usr/bin/my-tool.sh (code=exited, status=0/SUCCESS)
   Main PID: 705043 (code=exited, status=0/SUCCESS)
        CPU: 1ms

Dec 01 10:07:11 omen systemd[1]: Started my-service.service - mytool background service.
Dec 01 10:07:11 omen my-tool.sh[705043]: hello my-tool
Dec 01 10:07:11 omen systemd[1]: my-service.service: Deactivated successfully.
```

!!! Tip
    Check Remove using `apt remove <package name>`

    for `postrm` script functionality


# todo
- How to check systemd using docker or podman
- [check this post](https://medium.com/@javier-canizalez/leveraging-the-power-of-systemd-in-podman-containers-1b224a01664b)