#!/bin/bash

SESSION="mysession"

# Start a new session
tmux new-session -d -s $SESSION

# Split the window into four equal parts
tmux split-window -h  # Split vertically
tmux split-window -v  # Split bottom-left horizontally
tmux select-pane -R   # Move to the right pane
tmux split-window -v  # Split bottom-right horizontally

# Attach to session
tmux attach-session -t $SESSION
