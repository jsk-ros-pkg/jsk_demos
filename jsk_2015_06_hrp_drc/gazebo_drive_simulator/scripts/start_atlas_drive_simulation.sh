#! /bin/bash
function kill_rtmlaunch() {
    echo sending SIGKILL ...
    # sudo pkill -KILL -f roslaunch
    sudo pkill -KILL -f rtmlaunch
    sudo pkill -KILL -f omniNames
    return 0
}
trap "echo signal trapped.;kill_rtmlaunch; exit" 1 2 3 15

source ${HOME}/ros/hydro/devel/setup.sh
# `rospack find hrpsys_ros_bridge`/scripts/rtmlaunch hrpsys_gazebo_atlas atlas_v0_hrpsys_bringup.launch &
# sleep 60
# rosrun roseus roseus `rospack find gazebo_drive_simulator`/euslisp/atlas-drive-simulator-handle-joy.l "(main)"
`rospack find hrpsys_ros_bridge`/scripts/rtmlaunch gazebo_drive_simulator atlas_in_gazebo_drive_simulator.launch
