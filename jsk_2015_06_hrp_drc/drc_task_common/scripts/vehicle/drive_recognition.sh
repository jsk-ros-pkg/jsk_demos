#! /bin/bash

MODE="real"
FILENAME=""
GOAL_DIR=0

# parse argument
if [ $# -eq 1 ]; then
    GOAL_DIR=$1
elif [ $# -ge 2 ]; then
    MODE=$1
    GOAL_DIR=$2
fi
if [ $MODE = "rosbag" ]; then
    FILENAME=$2
    GOAL_DIR=0
    if [ $# -ge 3 ]; then
        GOAL_DIR=$3
    fi
fi

function kill_roslaunch() {
    echo sending SIGKILL to roslaunch...
    pkill -f rosrun roslaunch
    return 0
}

trap "echo signal trapped.;kill_roslaunch; exit" 1 2 3 15

if [ $MODE = "simulation" ]; then
    roslaunch gazebo_drive_simulator cheat_goal_direction.launch &
    roslaunch gazebo_drive_simulator multisense_sl_relay.launch & # for PointCloud
elif [ $MODE = "rosbag" ]; then
    rosparam set /use_sim_time true
    rosrun rviz rviz -sync -d $(rospack find gazebo_drive_simulator)/launch/atlas_drc_practice_task_1.rviz &
    roslaunch drc_task_common drive_rosbag_player.launch BAGFILE_NAME:=$FILENAME &
    # roslaunch drc_task_common car_center_tf_publisher.launch ROSBAG_MODE:="true" &
    # rosrun drc_task_common Magnetometer2Direction.py $GOAL_DIR & # deg
    # rostopic pub /drive/controller/step_on_flag std_msgs/Bool "true" &
else
    rosrun rviz rviz -sync -d $(rospack find gazebo_drive_simulator)/launch/atlas_drc_practice_task_1.rviz &
    # rosrun drc_task_common Magnetometer2Direction.py $GOAL_DIR & # deg
fi

sleep 5
roslaunch drc_task_common extract_obstacle_cloud.launch &
sleep 3
# roslaunch drc_task_common obstacle_detection.launch &
# roslaunch drc_task_common local_planner.launch &
roslaunch drc_task_common driver_assist.launch &
sleep 5
rosrun rqt_reconfigure rqt_reconfigure &


while true
do
    sleep 1 # wait termination
done
