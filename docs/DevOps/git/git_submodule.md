---
tags:
    - git
    - submodule
---

# Git Submodule

Submodules are repositories inside other Git repositories.




```bash
# Add a submodule to current repository at specified path
git submodule add -b master [URL to Git repo] [path]

```

```bash
# update all project submodules
git submodule update --init --recursive
```

### Remove a submodule
bash Script to remove a submodule from a git repository

```bash
submodule_path=$1

[ -d "$submodule_path" ] || (echo 'Specify valid submodule path as first parameter' && exit 1)

# Remove the submodule entry from .git/config
echo "Deinitializing submodule $submodule_path"
git submodule deinit -f $submodule_path

# Remove the submodule directory from the superproject's .git/modules directory
echo "Removing .git/modules for $submodule_path"
rm -rf .git/modules/$submodule_path

# Remove the entry in .gitmodules and remove the submodule directory located at path/to/submodule
echo "Removing files for $submodule_path"
git rm -rf $submodule_path
```