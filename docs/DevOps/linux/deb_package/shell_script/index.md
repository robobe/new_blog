---
tags:
    - debian
    - shell
    - dpkg-buildpackage
---

# Shell script debian package

pack minimal shell script to `debian` package using `dpkg-buildpackage`



```bash title="my-tool"
--8<-- "docs/DevOps/linux/deb_package/shell_script/code/my-tool/usr/local/bin/my-tool"
```


### debian folder
Using `dpkg-buildpackage` debian folder must contain few files:

- rules
- changelog
- control

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

### build
```bash
dpkg-buildpackage -us -uc
```


