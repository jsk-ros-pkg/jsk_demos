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
    if [ `tmux list-windows | grep $1 | sed -e 's/ //g' >/dev/null &2>1` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t fc
        tmux send-keys -t fc:$1 "$2" C-m
    fi
}

if `tmux has-session -t fc`; then
    echo -e "\e[1;33msession named fc already exists.\e[m"
else
    echo -e "\e[1;34mcreate new session named fc.\e[m"
    tmux new-session -d -s fc -n tmp
fi

tmux-newwindow executive "roslaunch drc_task_common fc_executive.launch"
tmux-newwindow stereo_preprocess "roslaunch drc_task_common stereo_preprocess.launch"
tmux-newwindow laser_preprocess "roslaunch drc_task_common laser_preprocess.launch"
tmux-newwindow lookat "roslaunch drc_task_common lookat.launch"
tmux-newwindow valve_recognition "roslaunch drc_task_common valve_recognition.launch"
tmux-newwindow drill_recognition "roslaunch drc_task_common drill_recognition.launch"
tmux-newwindow drill_recognition_for_button "roslaunch drc_task_common drill_recognition_for_button.launch"
tmux-newwindow drill_recognition_for_wall "roslaunch drc_task_common drill_recognition_for_wall.launch"
tmux-newwindow drill_recognition_for_put "roslaunch drc_task_common drill_recognition_for_put.launch"
tmux-newwindow drill_button_checker "roslaunch drc_task_common drill_button_checker.launch"
tmux-newwindow door_recognition "roslaunch drc_task_common door_unvisible_handle_recognition.launch"
tmux-newwindow debri_recognition "roslaunch drc_task_common debri_recognition.launch"
# tmux-newwindow panorama "roslaunch drc_task_common panorama.launch"
tmux-newwindow fisheye "roslaunch drc_task_common fisheye_lookat.launch"
tmux-newwindow locomotion "roslaunch drc_task_common locomotion.launch"
tmux-newwindow misc "roslaunch drc_task_common fc_misc.launch"
tmux-newwindow com "roslaunch drc_com_common field_computer_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
