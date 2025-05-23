---
tags:
    - linux
    - tmux
    - tmuxp
---

# tmuxp

Tmuxp is a session manager for tmux, allowing you to define tmux sessions using a YAML configuration

### install

```bash
sudo apt install tmux
pip install tmuxp
```

### config

```yaml title="my_session.yaml"
--8<-- "docs/DevOps/linux/session.yaml"
```

### usage
```bash
tmuxp load my_session.yaml
```

--- 

# tmux

## script
Open screen with 4 panes

```bash
--8<-- "docs/DevOps/linux/code/tmux_layout.sh"
```

!!! note "don't forget to chmod +x"
     
## Config

<details><summary>tmux.conf</summary>
```
# unbind
unbind C-b
unbind '"'
unbind %

# base1 numbering
set -g base-index 1
setw -g pane-base-index 1

#bind ctrl-a as a prefix
set-option -g prefix C-a
bind-key C-a send-prefix
# kill session
bind C-c kill-session

bind C-a run "tmux save-buffer - | xclip -i -sel clipboard"

# mouse
set -g mouse on


 # do like terminator
bind -n C-E split-window -h
bind -n C-S-Left resize-pane -L 3
bind -n C-S-Right resize-pane -R 3
bind -n C-S-Up resize-pane -U 3
bind -n C-S-Down resize-pane -D 3
bind -n C-O split-window -v

# switch panes using Alt-arrow without prefix (not working)
bind -n M-Left select-pane -L
bind -n M-Right select-pane -R
bind -n M-Up select-pane -U
bind -n M-Down select-pane -D

# Shift arrow to switch windows

bind n next-window
bind p previous-window

bind c new-window -c "#{pane_current_path}"

bind r source-file ~/.tmux.conf

# settings
```

</details>