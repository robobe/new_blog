---
tags:
    - ros
    - colcon
    - tips
    - settings
---

# Tips / Settings

## Colcon auto completion

[colcon site](https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion)

source `colcon_argcomplete` in your `.bashrc` file

```bash
# debian install location
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

---

## colcon defaults
[colcon read the docs](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml)

Set default option for `colcon` by create file `colcon_defaults.yaml` file at `workspace` directory.

```yaml title="default.yaml"
build:
  # Use symlink install to speed up builds
  symlink-install: true
  # Set build type (e.g., Release or Debug)
  cmake-args:
    - "-DCMAKE_BUILD_TYPE=Release"

test:
  # Run tests in parallel
  parallel-workers: 4

# Additional settings for colcon test and other commands
test-result:
  verbose: true

```