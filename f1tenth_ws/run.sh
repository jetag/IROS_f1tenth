#!/bin/bash

tmux send-keys "cd f1tenth_ws/ && gsrc && src" C-m
tmux split-window -v -p 50
tmux send-keys "cd f1tenth_ws/ && gsrc && src" C-m
tmux split-window -h -p 50
tmux send-keys "cd f1tenth_ws/ && gsrc && src" C-m
tmux split-window -v -p 50
tmux send-keys "cd f1tenth_ws/ && gsrc && src" C-m
