---
tags:
    - git
    - tips
---

# Git Tips and Settings

## prompt
Cool prompt generate by ChatAI

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