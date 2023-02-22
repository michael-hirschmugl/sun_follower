#!/bin/bash

tmux kill-session -t debug_session

tmux new -s debug_session -d

tmux send-keys -t debug_session "openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg" C-m

tmux new-window -t debug_session

tmux send-keys -t debug_session "cd cmake-build-debug" C-m
tmux send-keys -t debug_session "export PATH=$PATH:~/gcc-arm-none-eabi-10.3-2021.10/bin/" C-m
tmux send-keys -t debug_session "source ~/.zshrc" C-m
tmux send-keys -t debug_session "clear" C-m

tmux send-keys -t debug_session "arm-none-eabi-gdb -tui sun-follower.out" C-r C-m
sleep 1s
tmux send-keys -t debug_session C-r C-m
sleep 1s
tmux send-keys -t debug_session C-r C-m
sleep 1s
tmux send-keys -t debug_session "target extended-remote localhost:3333" C-r C-m
tmux send-keys -t debug_session "b main" C-r C-m
tmux send-keys -t debug_session "b 104" C-r C-m
tmux send-keys -t debug_session "r" C-r C-m
tmux send-keys -t debug_session "y" C-r C-m

tmux attach -t debug_session