#!/bin/bash

function getWID(){
    name=$1
    hint="$2"
    THE_PID=`ps aux | grep $name | grep "$hint" | grep -v grep | awk '{print $2}'`
    THE_WINDOW_ID=`wmctrl -lp | grep $THE_PID | awk '{print $1}'`
    echo $THE_WINDOW_ID
}

RVIZ_WID=`getWID drc_teleop_interface`
wmctrl -ir $RVIZ_WID -b add,above

