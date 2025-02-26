---
tags:
    - git
    - rebase
---

# GIT Rebase

![alt text](images/rebase_1.png)


```bash
#switch to feature branch
git checkout feature/cool

```
![alt text](images/rebase_2.png)


```bash
# from feature/cool
git checkout feature/cool
git rebase master
```


![](images/rebase_3.png)

```bash
# from master
git checkout master
git rebase feature/cool
```

![alt text](images/rebase_4.png)
## Reference
- [A better Git workflow with rebase](https://www.themoderncoder.com/a-better-git-workflow-with-rebase/)