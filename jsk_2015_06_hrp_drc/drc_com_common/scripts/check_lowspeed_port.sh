#!/bin/sh

for i in $(seq 0 10)
do
    start_port=$(expr $i \* 100)
    end_port=$(expr $(expr $i + 1) \* 100 - 1)
    if [ $end_port -gt 1024 ]; then
        end_port=1024
    fi
    ports=$(seq -s"," $start_port $end_port)
    sudo lsof -i:$ports
done


