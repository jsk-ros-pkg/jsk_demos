#! /bin/bash

USE_HANDLE="true"
if [ $# -ge 1 ]; then
    USE_HANDLE=$1
fi

source ${HOME}/ros/hydro/devel/setup.sh
rosrun gazebo_drive_simulator start_gazebo_drive_simulator.sh `rospack find drcsim_gazebo`/launch/vrc_task_1.launch /drc_vehicle_xp900 ${USE_HANDLE}

