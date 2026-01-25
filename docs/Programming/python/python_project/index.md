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

##### manual

```
version = "0.1.0"
```

##### dynamic
Get version from git tag
using  tools like `setuptools-scm`.

```ini title="pyproject.toml"
[build-system]
requires = ["setuptools", "setuptools-scm"]

[project]
dynamic = ["version"]

[tool.setuptools_scm]
```

!!! warning ""
    if there any checkout file the version include `dev` as suffix

#### usage

```python title="__init__"
from importlib.metadata import version

__version__ = version("my_package")
```

```python
import python_project

print(python_project.__version__)
```

---

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


<div class="grid-container">
    <div class="grid-item">
        <a href="debuild">
        <p>From source to debain using debuild</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="dpkg-deb">
        <p>From source to debain using dpkg-deb</p>
        </a>
    </div>
</div>

