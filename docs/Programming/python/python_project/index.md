---
tags:
    - python
    - debian
    - project
---

# under construction

# From python project to debian package

Create Minimal python project and create debian package from it

```
├── debian
│   ├── changelog
│   ├── compat
│   ├── control
│   ├── postinst
│   ├── postrm
│   └── rules
├── my_python_app
│   ├── app.py
│   └── __init__.py
├── setup.py
└── venv
```

## First step: Python project
Create python project struct
- venv
- setup.py
- project module
- register the project in the venv

```python
from setuptools import setup, find_packages

setup(
    name="my-python-app",
    version="0.1",
    packages=find_packages(),
    entry_points={
        "console_scripts": [
            "myapp=my_python_app.app:main"
        ]
    },
    install_requires=[],  # Add dependencies here if any
    author="Your Name",
    description="A simple Python app",
)
```

```python title="my_python_app/app.py"
def main():
    print("Hello, this is my Python app!")

    
if __name__ == "__main__":
    main()
```

```bash
# source into venv
pip install -e .
```

```bash
#execute `myapp` (application entry point)
myapp
```

---

## Step2: Debian 

```bash
sudo apt install build-essential devscripts debhelper dh-python
```

Create `debian` folder with this files
- debian/control
- debian/rules
- debian/changelog
- debian/postinst (optional)
- debian/postrm (optional)


### control

```
Source: my-python-app
Section: utils
Priority: optional
Maintainer: Your Name <your.email@example.com>
Build-Depends: debhelper (>= 9), dh-python, python3-all
Standards-Version: 4.5.0
Homepage: <optional URL>

Package: my-python-app
Architecture: all
Depends: python3 (>= 3.5), ${misc:Depends}, ${python3:Depends}
Description: A simple Python application
 This is a test package for my Python app.
```

### rules

!!! note "executable permission"
     

```make
#!/usr/bin/make -f
%:
	 dh $@ --with python3 --buildsystem=pybuild

```

!!! tip "check for tabs"

    ```
    cat -t debian/rules
    #
    ^I as tab and not spaces
    ```

### changelog

```bash
dch --create --package my-python-app -v 0.1-1 "Initial release"
```

#### Format

```title="template"
<package-name> (<version>) <distribution>; urgency=<urgency>

  * <change description>
  * <another change>

 -- <maintainer-name> <email>  <timestamp>
```

```title="example"
my-python-app (1.0-1) unstable; urgency=medium

  * Initial release.
  * Added feature X for better performance.
  * Fixed bug in module Y.

 -- John Doe <john@example.com>  Mon, 17 Mar 2025 12:34:56 +0000
```

**my-python-app (1.0-1) unstable; urgency=medium**
- package name: my-python-app
- version: (1.0-1)
    - 1.0: upstream (code version)
    - -1: Debian version (change in the rules and control files)
- distribution: 
    - unstable → Default for new packages.
    - stable → For official stable releases.
    - testing → For testing before stable.
    - experimental → For experimental features.
- urgency:
    - low → No hurry.
    - medium → Normal (default).
    - high → Important security fixes.
    - emergency → Critical fixes, immediate attention.
### compat

```
13
```

### postinst

```bash
#!/bin/sh
set -e  # Exit on any error

# Example: Create a config directory for your Python app
if [ "$1" = "configure" ]; then
    mkdir -p /etc/my-python-app
    echo "Configuration directory created."
fi

exit 0
```

### postrm

```bash
#!/bin/sh
set -e  # Exit on any error

# Example: Remove the config directory on purge
if [ "$1" = "purge" ]; then
    if [ -d /etc/my-python-app ]; then
        rm -rf /etc/my-python-app
        echo "Configuration directory removed."
    fi
fi

exit 0
```

---

## Step3: build the package

```
debuild -us -uc -b
```

!!! note "deb files"
    The deb file create in the parent folder
     

## Step4: Add user input during installation

```
sudo apt install debconf debconf-utils
```

## TODO


## debian/changelog?

The debian/changelog file is a required part of a Debian package. It’s a human-readable log of changes made to the package across its versions. It follows a specific format and is used by tools like dpkg and apt to display package history and determine version ordering. It’s also critical for maintaining a package in a Debian repository.
Format of debian/changelog

```bash
my-python-app (0.1-1) unstable; urgency=medium #(1)

  * Initial release of my Python app. #(2)

 -- Your Name <your.email@example.com>  Mon, 17 Mar 2025 12:00:00 +0000 #(3)
```

1. First Line:
        my-python-app: Package name.
        (0.1-1): Version (explained below).
        unstable: Target distribution (e.g., unstable, stable, or a codename like bookworm).
        urgency=medium: Priority of the update (e.g., low, medium, high, critical).

2. Change List:
        Bullet points (*) describing what changed in this version.
3. Signature Line:
        Author’s name, email, and timestamp.



### What is dch?

dch (Debian CHangelog) is a command-line tool from the devscripts package that simplifies editing debian/changelog. It helps you:

- Create a new changelog file.
- Add new version entries.
- Update timestamps and metadata.

```
sudo apt install devscripts
```


#### Create a New Changelog:
```bash
dch --create --package my-python-app -v 0.1-1 "Initial release"
```


```bash
dch -v 0.2-1 "Added new feature X"
```

Edit Manually:
```bash
dch -e
#    Opens the changelog in your default editor (e.g., nano or VS Code if configured).
```

#### Versioning: What Does -[number] Mean?

The version in debian/changelog (e.g., 0.1-1) follows Debian’s versioning scheme: [upstream_version]-[debian_revision].

- Upstream Version (0.1):
    This is the version of your software (e.g., from setup.py in your Python project).
    You increment this when you make changes to the actual code (e.g., 0.1 to 0.2 for a new feature).
- Debian Revision (-1):
    This is the packaging version, incremented when you change the Debian packaging (e.g., fix a bug in debian/rules or postinst) without changing the upstream code.
    Example: If you release 0.1-1 and later fix a packaging issue, the next version would be 0.1-2.

Rules for the - and Number

    Initial Release: Start with -1 (e.g., 0.1-1).
    Packaging Changes Only: Keep the upstream version the same and increment the revision (e.g., 0.1-1 → 0.1-2).
    New Upstream Release: Increment the upstream version and reset the revision to -1 (e.g., 0.1-1 → 0.2-1).
    No - Needed: If you’re not maintaining separate upstream and Debian versions (common for simple projects), you can omit the revision (e.g., 0.1), but including it (e.g., 0.1-1) is standard practice.

Examples

    0.1-1: First Debian package of upstream version 0.1.
    0.1-2: Second packaging attempt of upstream 0.1 (e.g., fixed a typo in debian/control).
    0.2-1: New upstream version 0.2, first Debian package.

### Git and debian package

```
sudo apt install git-buildpackage
```

```
git-dch --release --auto --git-author
```

- --release: Marks this as a full release.
- --auto: Generates entries from commit messages since the last tag.
- --git-author: Uses Git commit authors in the changelog.