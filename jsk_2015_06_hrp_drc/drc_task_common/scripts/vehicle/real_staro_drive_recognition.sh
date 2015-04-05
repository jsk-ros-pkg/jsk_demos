#! /bin/bash

function kill_roslaunch() {
    echo sending SIGKILL to roslaunch...
    sudo pkill -f roslaunch rosrun
    return 0
}

trap "echo signal trapped.;kill_roslaunch; exit" 1 2 3 15


# rosparam set use_sim_time true
rosrun rviz rviz &
sleep 3

# roslaunch gazebo_drive_simulator staro_multisense_relay.launch &
roslaunch jsk_data point_cloud_reconstruction_from_multisense.launch &
roslaunch drive_recognition multisense_rosbag_static_tf_publisher.launch &

roslaunch drive_recognition extract_obstacle_cloud.launch &
# roslaunch drive_recognition obstacle_detection.launch &

sleep 10

#temporary
#rostopic pub /staro_drive/pedal_state std_msgs/Bool "True" # for local_planner.launch
rostopic pub /staro_drive/operation/flag/handle std_msgs/Bool "False" & # for local_planner_mochikae.launch
rosrun drive_recognition Magnetometer2Direction.py 15 &

while true
do
    sleep 1 # wait termination
done
