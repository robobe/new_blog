---
title: Python project from source to DEB using dpkg-deb
tags:
  - python
  - debian
  - dpkg-deb
---

## dpkg-deb
Is a low-level debian tool it turn a filesystem tree into debian package


Using `dpkg-deb` util to create debian package from python project (wrap the **whl** file create by `python3 -m build`)

!!! tip "todo"
  extract the whl file into the debain filesystem and the pack , 
  remove the pip install from posinst
    


### Demo
[demo code ](https://github.com/robobe/python_project/tree/build_deb){:target="_blank"}
My basic idea is create project DEBIAN folder the include all the file that i need to pack include the **whl** file that create using `python -m build`

- Create root folder named `python3-<project-name>`
- Create DEBIAN folder
- Add **control** file under DEBIAN folder
- Using `python -m build --outdir` to place the dir under the **project debian** folder (dist sub folder)
- Add `postinst` and `postrm` to install the `whl` file when we install the package
- Set execte permission to `postinst` and `postrm` and `644` to `control` file
- Create `debs` folder under project root for deb output
- Run `dpkg-deb python3-<project-name> debs` 
- Check the debian using `dpkg -I` and `dpkg -c`


#### Project

```
├── pyproject.toml
├── python3-python-project
│   ├── DEBIAN
│   │   ├── control
│   │   ├── postinst
│   │   └── postrm
│   ├── dist
│   │   ├── my_package-0.0.1-py3-none-any.whl
│   │   └── my_package-0.0.1.tar.gz
├── python_project
│   ├── app.py
│   └── __init__.py
└── README.md

```


```title="control"
Package: python3-python-project
Version: 1.0.0
Section: python
Priority: optional
Architecture: all
Depends: python3, python3-pip
Maintainer: Your Name <you@email.com>
Description: My Python project packaged as a Debian package
```

```bash title="postint"
#!/bin/sh
set -e

pip3 install --no-deps /dist/my_package-0.0.1-py3-none-any.whl

echo "Installation complete."

exit 0
```

```bash title="postrm"
#!/bin/sh
set -e

echo "Removing Python package..."

exit 0
```
---

## TODO
- i know that run pip from postinst it`s bead idea, 