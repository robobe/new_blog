---
tags:
    - git
    - cherrypick
---

# Git Cherrypick

Git cherry-pick command allows you to **apply a specific commit from one branch onto another**

```bash
# list commit from other branch
git log <branch name> --oneline

# cherry
# from current branch cheery  commit from another branch
git cherry-pick <commit hash>
```

### cherry pick multiple commits

```bash
git cheery-pick <hash1> <hash2>
```

!!! tip "apply order"
    cherry pick apply changes by the hash order
     