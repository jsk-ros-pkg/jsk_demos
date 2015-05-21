for i in 1 2 3 4 5 6 7
do
    for x in 2 1 0
    do
        for y in 2 1 0
        do
            for z in 2 1 0
            do
                for r in 0 1 2 3 4 5 6 7
                do 
                    rossetlocal
                    rosparam set i ${i}
                    rosparam set ix ${x}
                    rosparam set iy ${y}
                    rosparam set iz ${z}
                    rosparam set ir ${r}
                    rosrun drc_task_common generate-drill-button-angle.l
                done
            done
        done
    done
done
