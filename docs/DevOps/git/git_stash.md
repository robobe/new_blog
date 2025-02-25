---
tags:
  - git
  - stash
---

# Git Stash

Git stash temporarily saves your local changes without committing them, allowing you to work on something else and then reapply those changes later.

| Command                                       | Description                          |
| --------------------------------------------- | ------------------------------------ |
| git stash                                     | Add track uncommit file to the stash |
| git stash pop                                 | Apply and remove from stash          |
| git stash apply                               | Apply without remove                 |
| git stash apply@{1}                           | Apply specific stash                 |
| git stash list                                | List all stash entries               |
| git stash clear                               | clear entries                        |
| git stash push -m "stash message"             | push with message                    |
| git stash push -m "stash message" -- filename | push specific file with message      |

## Reference

- [ Git Stash In 5 Minutes ](https://www.youtube.com/watch?v=lH3ZkwbVp5E)
