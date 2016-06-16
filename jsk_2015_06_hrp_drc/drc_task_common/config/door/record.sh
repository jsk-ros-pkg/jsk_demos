#!/bin/bash

files=$(find ~/ueda_bags/2016-01-08-1 -name '*bag')

for f in $files
do
    rosrun jsk_tools kill_after_seconds.py 150 roslaunch record.launch bag:=$f &
    for job in $(jobs -p)
    do
        wait $job
    done
done
