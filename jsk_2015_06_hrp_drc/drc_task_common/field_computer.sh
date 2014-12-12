#!/usr/bin/env bash

tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g'` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t fc
        tmux send-keys -t fc:$1 "$2" C-m
    fi
}

if [ $# -gt 0 ]; then
    if [ $1 = "--kill" -o $1 = "k" ]; then
        tmux kill-session -t fc
        exit 0
    elif [ $1 = "--attach" -o $1 = "a" ]; then
        tmux a -t fc
        exit 0
    fi
fi

if `tmux has-session -t fc`; then
    echo -e "\e[1;33msession named fc already exists.\e[m"
else
    echo -e "\e[1;34mcreate new session named fc.\e[m"
    tmux new-session -d -s fc -n tmp
fi

tmux-newwindow task_fc "roslaunch drc_task_common field_computer_main.launch"
tmux-newwindow com_fc "roslaunch drc_com_common field_computer_com.launch"
tmux send-keys -t fc:tmp "exit" C-m
