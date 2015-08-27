#!/usr/bin/env bash

CMDNAME=$(basename $0)
FC_IP=localhost
OCS_IP=localhost
while getopts hf:o:ka OPT
do
    case $OPT in
        "f") FC_IP="$OPTARG";;
        "o") OCS_IP="$OPTARG";;
        "k") tmux kill-session -t fc; exit;;
        "a") tmux a -t fc; exit;;
        "h") echo "Usage: $CMDNAME [-f FC_IP] [-o OCS_IP]"; exit;;
    esac
done


tmux-newwindow() {
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g' >/dev/null 2>&1` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t fc
        tmux send-keys -t fc:$1 "$2" C-m
    fi
}

if `tmux has-session -t fc`; then
    echo -e "\e[1;33msession named fc already exists.\e[m"
    exit 1
else
    echo -e "\e[1;34mcreate new session named fc.\e[m"
    tmux new-session -d -s fc -n tmp
fi
if [ "$ROBOT" = "JAXON" -o "$ROBOT" = "JAXON_RED" ]; then
    tmux-newwindow multisense "sleep 1; roslaunch jaxon_ros_bridge jaxon_multisense_remote.launch"
fi
tmux-newwindow executive "sleep 1; roslaunch drc_task_common fc_executive.launch"
tmux-newwindow lookat "sleep 1; roslaunch drc_task_common lookat.launch"
tmux-newwindow fisheye "sleep 5; roslaunch drc_task_common fisheye_lookat.launch"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common locomotion.launch"
# tmux-newwindow locomotion_planner "sleep 5; roslaunch drc_task_common locomotion_planner.launch"
tmux-newwindow misc "sleep 5; roslaunch drc_task_common fc_misc.launch"
tmux-newwindow com "sleep 5; roslaunch drc_com_common field_computer_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
