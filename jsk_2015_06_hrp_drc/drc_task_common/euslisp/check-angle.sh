for i in 0 30 60 90 120 150 180 210 240 270 300 330
do
    rosparam set angle ${i}
    rosrun drc_task_common drill-button-angle-checker.l
done