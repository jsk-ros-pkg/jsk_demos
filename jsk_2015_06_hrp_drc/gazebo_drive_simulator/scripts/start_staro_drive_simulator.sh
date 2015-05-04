#! /bin/bash

function kill_roslaunch() {
    echo sending SIGKILL to roslaunch...
    sudo pkill -f roslaunch rosrun
    return 0
}

trap "echo signal trapped.;kill_roslaunch; exit" 1 2 3 15

USE_HANDLE="true"

source ${HOME}/ros/hydro/devel/setup.sh
source `rospack find hrpsys_gazebo_tutorials`/setup.sh
rosrun roseus roseus `rospack find gazebo_drive_simulator`/euslisp/staro-gazebo-simulator-sitting.l
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{position: {x: 0.1, y: -0.3, z: -0.05}}' # ride to drc_vehicle
rostopic pub --once /drc_vehicle_xp900/hand_brake/cmd std_msgs/Float64 '{ data : 0 }' # disable hand brake
rosnode kill /robot_pose_ekf
roslaunch gazebo_drive_simulator robot_pose_ekf_gazebo_drive_simulator.launch &

roslaunch gazebo_drive_simulator staro_multisense_relay.launch &

roslaunch drc_task_common extract_obstacle_cloud.launch &
roslaunch drc_task_common obstacle_detection.launch &
# rosrun drive_recognition CalculateVelocityFromOdometry.py &
# roslaunch drive_recognition cheat_goal_direction.launch &
roslaunch gazebo_drive_simulator polaris_interactive_marker.launch &
# rosrun roseus roseus `rospack find drive_recognition`/euslisp/staro-look-around.l "(demo-main)" &
sleep 20
rostopic pub --once /staro_look_around/neck_p_angle std_msgs/Float64 '{ data : 15.0 }' # disable hand brake

while true
do
    sleep 1 # wait termination
done
