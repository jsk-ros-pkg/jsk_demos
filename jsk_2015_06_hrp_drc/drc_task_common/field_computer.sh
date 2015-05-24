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

tmux-newwindow executive "sleep 1; roslaunch drc_task_common fc_executive.launch"
tmux-newwindow stereo_preprocess "sleep 1; roslaunch drc_task_common stereo_preprocess.launch"
tmux-newwindow laser_preprocess "sleep 1; roslaunch drc_task_common laser_preprocess.launch"
tmux-newwindow lookat "sleep 1; roslaunch drc_task_common lookat.launch"
tmux-newwindow valve_recognition "sleep 1; roslaunch drc_task_common valve_recognition.launch"
tmux-newwindow drill_recognition "sleep 3; roslaunch drc_task_common drill_recognition.launch"
tmux-newwindow drill_recognition_for_button "sleep 3; roslaunch drc_task_common drill_recognition_for_button.launch"
tmux-newwindow drill_recognition_for_wall "sleep 3; roslaunch drc_task_common drill_recognition_for_wall.launch"
tmux-newwindow drill_recognition_for_put "sleep 3; roslaunch drc_task_common drill_recognition_for_put.launch"
tmux-newwindow drill_button_checker "sleep 3; roslaunch drc_task_common drill_button_checker.launch"
tmux-newwindow door_recognition "sleep 5; roslaunch drc_task_common door_unvisible_handle_recognition.launch"
# tmux-newwindow debri_recognition "sleep 5; roslaunch drc_task_common hose_connect.launch"
# tmux-newwindow debri_recognition "sleep 5; roslaunch drc_task_common hose_grasp.launch"
# tmux-newwindow debri_recognition "sleep 5; roslaunch drc_task_common debri_recognition.launch"
# tmux-newwindow panorama "roslaunch drc_task_common panorama.launch"
tmux-newwindow fisheye "sleep 5; roslaunch drc_task_common fisheye_lookat.launch"
tmux-newwindow locomotion "sleep 5; roslaunch drc_task_common locomotion.launch"
tmux-newwindow vehicle "roslaunch drc_task_common vehicle_field_computer_main.launch USE_COM:=false ROBOT:=${ROBOT,,}"
tmux-newwindow misc "sleep 5; roslaunch drc_task_common fc_misc.launch"
tmux-newwindow com "sleep 5; roslaunch drc_com_common field_computer_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t fc:tmp "exit" C-m
tmux a -t fc
