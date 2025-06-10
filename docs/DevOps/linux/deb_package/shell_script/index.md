---
tags:
    - debian
    - dch
    - dpkg-buildpackage
    - fakeroot
    - rules
    - deb package
---

# Shell script debian package

pack minimal shell script to `debian` package using `fakeroot` / `dpkg-buildpackage`

!!! note "fakeroot / dpkg-buildpackage"
     When i use `dpkg-buildpackage` the build process end with error that for now i don't found a solution  
     the `fakeroot debian/rules binary run the build process without error

```
.
├── debs
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



```bash title="scripts/my-tool"
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/my-tool/usr/local/bin/my-tool"
```


### debian folder
Using `fakeroot`/`dpkg-buildpackage` debian folder must contain few files:

- rules
- changelog
- control
- my-tool.install


#### my-tool.install
Mapping file from project to filesystem location for example

```init title="debian/my-tool.install"
scripts/my-tools.sh /usr/local/bin
```


!!! tip "external file"
    We can place file from external project folder relative to project root

    ```
    ../external /usr/local/bin
    ```
     

#### changelog
`dch` command create and update changelog file
```bash
dch --create -v 0.0.1 --package my-tool "v 0.0.1"
dch -v 0.0.1 "simple script"
```

!!! note version
    Using `dch` command we update the version 
    `my-tool (0.0.2) UNRELEASED; urgency=medium`

    This version use to set the debian version

    ```bash
    dch -v 0.0.2 "simple script ver 2"
    ```

    <details>
        <summary>changelog</summary>
    
    ```bash title="changelog"
    my-tool (0.0.2) UNRELEASED; urgency=medium

    * version 0.0.1
    * simple script ver 2

    -- user <user@lap>  Tue, 29 Apr 2025 21:14:15 +0300
    ```
    </details>
    
    


#### control

```
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/my-tool/debian/control"
```

#### rules

```
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/my-tool/debian/rules"
```

##### change deb output path (debs folder must be exists)

```make
override_dh_builddeb:
	dh_builddeb --destdir=$(CURDIR)/debs
```

##### set permissions

```
override_dh_fixperms:
	dh_fixperms
	chmod 777 debian/my-tool/usr/local/bin/demo.sh
```


---

### build
```bash
fakeroot debian/rules binary
# or 
dpkg-buildpackage -us -uc
```

### clean
```bash
fakeroot debian/rules clean
```

---

## VSCode

### tasks.json
- build using `fakeroot debian/rules binary`
- clean `fakeroot debian/rules clean`
- update change log using `dch`

<details>
    <summary>tasks.json</summary>

```json
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/my-tool/.vscode/tasks.json"
```
</details>


!!! bug ""
    the demo use `git log -1 --pretty=format:'* %s')` as demo to get changes from last commit
    it's batter to get the commits from last tag using `git log v1.2.0..v1.3.0 --pretty=format:"* %s"`
     

### setting.json
Replace indentation from space to tab when using sheelscript

!!! tip ""
    - mark rule as shellscript if fail on autodetect
    - check if associate it with makefile still the indention rule for shellscript working

    ```json title="settings.json"
    "files.associations": {
        "rules": "makefile"
    }
    ```

<details>
    <summary>settings.json</summary>

```json
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/my-tool/.vscode/settings.json"
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