#! /bin/bash

function kill_roslaunch() {
    echo sending SIGKILL to roslaunch...
    sudo pkill -f ros
    return 0
}

trap "echo signal trapped.;kill_roslaunch; exit" 1 2 3 15

SIMULATION_LAUNCH=`rospack find drcsim_gazebo`/launch/vrc_task_1.launch
CMD_NAMESPACE=/drc_vehicle
USE_HANDLE="true"

# parse arguments
if [ $# -ge 3 ]; then
    SIMULATION_LAUNCH=$1
    CMD_NAMESPACE=$2
    USE_HANDLE=$3
fi

source ${HOME}/ros/hydro/devel/setup.sh
roslaunch gazebo_drive_simulator gazebo_drive_simulator.launch SIMULATION_LAUNCH:=${SIMULATION_LAUNCH} CMD_NAMESPACE:=${CMD_NAMESPACE} USE_HANDLE:=${USE_HANDLE} &
sleep 25
rostopic pub --once /drc_world/robot_exit_car geometry_msgs/Pose '{}' # exit from wall
sleep 2
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{position: {y: -0.6, z: -0.05}}'
rostopic pub --once ${CMD_NAMESPACE}/hand_brake/cmd std_msgs/Float64 '{ data : 0 }' # disable hand brake

while true
do
    sleep 1 # wait termination
done
