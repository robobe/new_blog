---
tags:
    - git
    - git-diff
---

# Git diff

|  git command |   |
|---|---|
| git diff  | shows the changes that you have made in your working directory but not yet staged for commit   <br> It compares the files you have modified in your working directory with the **last committed** version of those files.  |
| git diff --stage | shows the differences between the **staging area (index)** and the last commit  |
| git diff ref1..ref2 | compare the changes between two different references (e.g., commits, branches, tags)   |

## other tips
### compare specific file

```bash
git diff <ref1>..<ref2> -- <file-path>


```

