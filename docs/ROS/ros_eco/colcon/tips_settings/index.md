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

Set default option for `colcon` by create file `defaults.yaml` at `~/.colcon/` directory.

```yaml title=default.yaml
{
    "build": {
        "cmake-args": ["-DCMAKE_BUILD_TYPE=RelWithDebInfo"],
        "merge-install": true,
        "symlink-install": true
    },
    "test": {
        "merge-install": true,
    },
}