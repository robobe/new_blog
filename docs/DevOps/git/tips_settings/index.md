---
tags:
    - git
    - tips
---

# Git Tips and Settings

## prompt
Effective prompt with git branch name

```bash
function git-branch-name { 
    git symbolic-ref HEAD 2>/dev/null | cut -d"/" -f 3- 
} 
    
function git-branch-prompt { 
    local branch=`git-branch-name` 
    if [ $branch ]; then printf " [%s]" $branch; fi 
} 
    
    
    
function re-prompt { 
    PS1="\u@\h \[\033[0;36m\]\W\[\033[0m\]\[\033[0;32m\]\$(git-branch-prompt)\[\033[0m\] \$ " 
} 
    
PROMPT_COMMAND=re-prompt 
    
trap 're-prompt' DEBUG 

```

## Git branching

### Sync remote branch with local branch

Remove all remote branches that no longer exist on the remote repository.
```bash
git fetch --prune
```


---

## git sparse-checkout
is a Git feature that allows you to partially clone a repository, downloading only specific folders or files instead of the entire project.

```bash
git clone --depth 1 --filter=blob:none --sparse <url>
git sparse-checkout set <folders to download separate by space>
```