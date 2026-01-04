---
title: Pack python project as wheel and debian package
tags:
    - python
    - debian
    - pyproject.toml
---

**pyproject.toml** is a standardized file (PEP 518, PEP 621) used to configure Python packaging, building, and tools.

```
my_package/
├── pyproject.toml
├── README.md
└── my_package/
    ├── __init__.py
    └── main.py
```

```toml
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "myproject"
version = "0.1.0"
description = ""
readme = "README.md"
requires-python = ">=3.8"
```


### build-system
Defines how your project is built.
This tells Python which tools to use when running `pip install` or `python -m build`.
`setuptools` is the classic standard and widely use

other build tools

- Poetry
- Hatch
- flit

---

### python build
**How to turn your source code into an installable package**

```
pip install build
```

```bash title="usage"
python -m build

# create wheel in `dist` folder
```


!!! tip
    use `--no-isolation` flag to disabled creating isolated virtual environment during build

---

### project metadata (PEP 621)
The [project] section in pyproject.toml that describes your Python package:

- name
- version
- dependencies
- dependencies
- entry points
- requires-python
- and more

#### version

Set manual
```
version = "0.1.0"
```

or dynamic using  tools like `setuptools-scm`.
```
dynamic = ["version"]
```

#### dependencies

```toml
dependencies = [
    "requests>=2.28",
    "numpy",
]
```

#### more project settings

```toml
[project.optional-dependencies]
dev = ["pytest", "black", "flake8"]

[project.urls]
Homepage = "https://github.com/alice/coolapp"
Docs = "https://coolapp.readthedocs.io"

[project.scripts]
coolapp = "my_package:main"
```

- **[project.optional-dependencies]**: install optional package `pip install my_package[dev]`
- **[project.scripts]**: entry point/CLI command run `coolapp` to run application after install

---


[Build Your First Python Package with pyproject.toml](https://medium.com/@codebyteexplorer/build-your-first-python-package-with-pyproject-toml-19e2119edbca)



---

# From wheel to deb

| Tool              | Uses `.install` | Easy  | Policy-correct | Best for      |
| ----------------- | --------------- | ----- | -------------- | ------------- |
| dpkg-deb          | ❌               | ⭐⭐⭐⭐  | ❌              | quick hacks   |
| dpkg-buildpackage | ✅               | ⭐⭐    | ✅              | real Debian   |
| debuild           | ✅               | ⭐⭐    | ✅              | same as above |
| stdeb             | ⚠️              | ⭐⭐⭐   | ⚠️             | Python-only   |

## dpkg-deb
Using `dpkg-deb` util to create debian package from python project (wrap the **whl** file create by `python3 -m build`)


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