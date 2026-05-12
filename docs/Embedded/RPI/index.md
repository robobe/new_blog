---
tags:
    - rpi
    - raspberry
---

# Raspberry Pi

<div class="grid-container">
     <div class="grid-item">
        <a href="ssh_vscode_remote">
            <p>VSCode Remote</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="gpio">
            <p>GPIO</p>
        </a>
    </div>
    <div class="grid-item">
        <a href="uart">
            <p>uart</p>
        </a>
    </div>
    <!-- <div class="grid-item">
        <a href="dev">
            <p>VSCode dev</p>
        </a>
    </div> -->
   
</div>

## 

### Prompt
- Add to .bashrc
- Change prompt when user connect from ssh session

```bash title="prompt"
if [ -n "$SSH_CONNECTION" ]; then
    if [ "$EUID" -eq 0 ]; then
        PS1='🍓\[\e[1;41m\][ROOT SSH \u@\h]\[\e[0m\] \w # '
    else
        PS1='🍓\[\e[1;31m\][SSH \u@\h]\[\e[0m\] \w \$ '
    fi
fi

```