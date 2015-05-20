#!/usr/bin/env bash

DRC_TASK_COMMON_LAUNCHES="fc_executive.launch stereo_preprocess.launch laser_preprocess.launch lookat.launch valve_recognition.launch \
drill_recognition.launch drill_recognition_for_button.launch drill_recognition_for_wall.launch \
drill_recognition_for_put.launch drill_button_checker.launch door_recognition.launch \
debri_recognition.launch panorama.launch locomotion.launch fc_misc.launch"
DRC_COM_COMMON_LAUNCHES="field_computer_com.launch"

CMDNAME=$(basename $0)
FC_IP=localhost
OCS_IP=localhost
while getopts hf:o: OPT
do
    case $OPT in
        "f") FC_IP="$OPTARG";;
        "o") OCS_IP="$OPTARG";;
        "h") echo "Usage: $CMDNAME [-f FC_IP] [-o OCS_IP]";;
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

for l in $DRC_TASK_COMMON_LAUNCHES
do
    tmux-newwindow $(basename $l .launch)_fc "roslaunch drc_task_common $l"
done
tmux-newwindow field_computer_com_ocs "roslaunch drc_com_common field_computer_com.launch FC_IP:=${FC_IP} OCS_IP:=${OCS_IP}"
tmux send-keys -t fc:tmp "exit" C-m
