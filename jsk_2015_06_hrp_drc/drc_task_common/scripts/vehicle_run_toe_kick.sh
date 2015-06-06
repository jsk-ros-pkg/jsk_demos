#! /bin/bash

rosnode kill /drive_controller
sleep 5
roslaunch drc_task_common vehicle_run_toe_kick.launch
