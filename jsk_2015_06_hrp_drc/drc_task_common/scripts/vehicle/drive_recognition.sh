#! /bin/bash

MODE="real"

# parse argument
if [ $# -ge 1 ]; then
    MODE=$1
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
    rosparam set use_sim_time true
    rosrun rviz rviz -sync -d $(rospack find gazebo_drive_simulator)/launch/atlas_drc_practice_task_1.rviz &
    roslaunch jsk_data point_cloud_reconstruction_from_multisense.launch &
    roslaunch drc_task_common car_center_tf_publisher.launch ROSBAG_MODE:="true" &
    rosrun drc_task_common Magnetometer2Direction.py 15 & # deg
    rostopic pub /drive/controller/step_on_flag std_msgs/Bool "true" &
else
    rosrun rviz rviz -sync -d $(rospack find gazebo_drive_simulator)/launch/atlas_drc_practice_task_1.rviz &
    rosrun drc_task_common Magnetometer2Direction.py 0 & # deg
fi

sleep 5
roslaunch drc_task_common extract_obstacle_cloud.launch &
sleep 3
# roslaunch drc_task_common obstacle_detection.launch &
roslaunch drc_task_common local_planner.launch &
roslaunch drc_task_common driver_assist.launch &
sleep 5
rosrun rqt_reconfigure rqt_reconfigure &


while true
do
    sleep 1 # wait termination
done
